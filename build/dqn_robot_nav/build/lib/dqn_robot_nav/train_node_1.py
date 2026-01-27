import rclpy
import os
import numpy as np
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor

def main(args=None):
    rclpy.init(args=args)
    
    # Directorio para guardar modelos
    save_dir = os.path.join(os.getcwd(), 'saved_models')
    os.makedirs(save_dir, exist_ok=True)
    
    # Inicializar componentes
    env = TurtleBot3Env()
    state_proc = StateProcessor(n_lidar_bins=10)
    
    # Esperar a que ROS conecte y recibamos datos del láser
    print("Esperando conexión con simulador...")
    while env.scan_data is None:
        rclpy.spin_once(env, timeout_sec=0.1)
    
    # Dimensiones: 10 del lidar + 2 de la meta (distancia, angulo)
    state_size = 12
    # Obtenemos el tamaño de acción directamente del entorno
    action_size = env.action_size
    
    # --- AQUÍ ESTABA EL ERROR ---
    # Inicializamos el agente SOLO con los tamaños, sin hiperparámetros extra
    agent = DQNAgent(state_size, action_size)
    
    episodes = 500
    
    print(f"Iniciando entrenamiento con Estado: {state_size} inputs, Acciones: {action_size} outputs")

    for e in range(episodes):
        # Resetear entorno
        scan = env.reset()
        
        # Obtener estado inicial
        state = state_proc.get_state(scan, env.position, env.goal_position, env.yaw)
        
        total_reward = 0
        steps = 0
        
        while True:
            # 1. Elegir acción
            action = agent.act(state)
            
            # 2. Ejecutar acción
            next_scan, reward, done = env.step(action)
            
            # 3. Procesar siguiente estado
            next_state = state_proc.get_state(next_scan, env.position, env.goal_position, env.yaw)
            
            # 4. Guardar en memoria y entrenar
            agent.remember(state, action, reward, next_state, done)
            loss = agent.replay()
            
            state = next_state
            total_reward += reward
            steps += 1
            
            # Limite de pasos por episodio para seguridad
            if steps > 500:
                done = True
            
            if done:
                print(f"Episodio: {e+1}/{episodes} | Pasos: {steps} | Recompensa: {total_reward:.2f} | Epsilon: {agent.epsilon:.2f}")
                break
                
        # Guardar modelo cada 10 episodios
        if e % 10 == 0:
            save_path = os.path.join(save_dir, f"dqn_model_{e}.pkl")
            agent.save(save_path)
            # print(f"Modelo guardado en {save_path}")

    print("Entrenamiento finalizado.")
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()