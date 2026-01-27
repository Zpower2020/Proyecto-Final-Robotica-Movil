import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
import math
import time

class TurtleBot3Env(Node):
    """ROS2 Environment wrapper for Stage Simulator"""
    
    def __init__(self):
        super().__init__('turtlebot3_env')
        
        # --- Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', 
                                                 self.scan_callback, 10)
        
        # IMPORTANTE: Escuchamos la odometría "limpia" de tu wrapper
        self.odom_sub = self.create_subscription(Odometry, '/odom/sim',
                                                 self.odom_callback, 10)
        
        # IMPORTANTE: Llamamos a TU servicio de reset
        self.reset_client = self.create_client(Empty, 'reset_sim')
        
        # --- Variables de Estado ---
        self.scan_data = None
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.goal_position = (0.0, 0.0)
        
        # Definición de Acciones (Linear Vel, Angular Vel)
        self.actions = {
            0: (0.5, 0.0),   # Avanzar rápido
            1: (0.2, 0.0),   # Avanzar lento
            2: (0.0, 0.5),   # Girar Izquierda
            3: (0.0, -0.5),  # Girar Derecha
            4: (0.2, 0.3),   # Curva Izquierda
            5: (0.2, -0.3),  # Curva Derecha
        }
        self.action_size = len(self.actions)

    def scan_callback(self, msg):
        self.scan_data = list(msg.ranges)

    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Cuaternión a Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def step(self, action_idx):
        # 1. Ejecutar Acción
        linear, angular = self.actions[action_idx]
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)
        
        # 2. Esperar un poco para que la acción tenga efecto (Stage es rápido)
        # En Gazebo se usa spin_once, aquí un pequeño sleep ayuda a estabilizar
        time.sleep(0.05) 
        rclpy.spin_once(self, timeout_sec=0.0)
        
        # 3. Calcular Recompensa y Estado
        done = False
        reward = 0.0
        
        dist_to_goal = self.get_distance_to_goal()
        
        # --- Lógica de Recompensas ---
        
        # A) Choque
        if self.check_collision():
            reward = -100.0
            done = True
            print("💥 Choque!")
            
        # B) Meta alcanzada (umbral de 0.3m)
        elif dist_to_goal < 0.3:
            reward = 200.0
            done = True
            print("🏆 META ALCANZADA!")
            
        # C) Recompensa por acercarse
        else:
            # Recompensa base por sobrevivir
            reward = 0.1 
            
            # Bonus si nos acercamos
            if dist_to_goal < self.last_distance:
                reward += 1.0
            
            self.last_distance = dist_to_goal
            
        return self.scan_data, reward, done

    def reset(self):
        # 1. Parar el robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 2. Resetear simulador (Teletransporte inicial)
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio reset_sim no disponible')
            return [10.0] * 360
            
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # --- BUCLE DE SEGURIDAD DE SPAWN ---
        # Intentamos buscar una posición segura hasta 5 veces
        max_retries = 5
        self.scan_data = None
        
        for attempt in range(max_retries):
            # A. Generar meta aleatoria
            self.goal_position = (
                np.random.uniform(-3.0, 3.0),
                np.random.uniform(-3.0, 3.0)
            )
            
            # B. Esperar datos de láser frescos
            # Esperamos un poco para que Stage actualice
            for _ in range(10): 
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.scan_data is not None:
                    break
            
            # C. Verificar si hemos nacido chocados
            if self.check_collision():
                print(f"⚠️ Spawn inseguro (Intento {attempt+1}). Re-reseteando...")
                # Volvemos a llamar al servicio para que nos mueva a otro sitio (si el servicio aleatoriza)
                # O si el servicio solo resetea al 0,0, aquí deberíamos confiar en que el robot se mueva.
                # NOTA: Como reset_stage.py suele ponerlo en 0,0, 
                # asegurate de que el 0,0 del mapa esté libre.
                
                # Si tu reset_stage.py NO aleatoriza la posición del robot, 
                # el problema es que el robot nace siempre en el mismo sitio 
                # y la meta es lo que cambia.
                
                # Si el robot nace en (0,0) siempre, asegúrate que (0,0) no tenga obstáculos.
                pass
            else:
                # Si no hay colisión, es un spawn seguro
                break

        # Esperar un último momento para estabilizar
        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        self.last_distance = self.get_distance_to_goal()
        print(f"🏁 Reset completado. Meta: {self.goal_position}")
        
        if self.scan_data is None:
             return [3.5] * 360 # Retorno seguro por defecto
             
        return self.scan_data

    def get_distance_to_goal(self):
        return math.sqrt(
            (self.goal_position[0] - self.position[0])**2 + 
            (self.goal_position[1] - self.position[1])**2
        )

    def check_collision(self):
        if self.scan_data is None:
            return False
        # Si algo está a menos de 25cm, es choque (el robot mide ~20cm)
        min_dist = min([x for x in self.scan_data if not math.isnan(x)], default=10.0)
        return min_dist < 0.25