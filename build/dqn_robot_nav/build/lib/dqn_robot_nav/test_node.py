import rclpy
import os
import numpy as np
import time
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor

def main(args=None):
    rclpy.init(args=args)
    
    env = TurtleBot3Env()
    state_proc = StateProcessor(n_lidar_bins=10)
    
    # 1. Configuración
    state_size = 12
    action_size = env.action_size
    agent = DQNAgent(state_size, action_size)
    
    # 2. CARGAR EL MEJOR MODELO
    # Cambia el número por el archivo .pkl que tengas en la carpeta.
    model_path = os.path.join(os.getcwd(), 'saved_model_final', 'dqn_model_1.pkl')
    
    if os.path.exists(model_path):
        print(f"Cargando modelo para evaluación: {model_path}")
        agent.load(model_path)
        # IMPORTANTE: Apagar la exploración para el examen
        agent.epsilon = 0.0 
    else:
        print(f"¡ERROR! No se encontró el modelo en: {model_path}")
        print("Verifica que la carpeta 'saved_models' exista y tenga el archivo.")
        return

    # 3. Correr 100 pruebas VÁLIDAS
    total_episodes = 100
    success_count = 0
    current_episode = 0 # Usamos contador manual
    
    print(f"Iniciando Fase de Evaluación ({total_episodes} episodios válidos)...")

    while current_episode < total_episodes:
        scan = env.reset()
        state = state_proc.get_state(scan, env.position, env.goal_position, env.yaw)
        done = False
        steps = 0
        
        while not done and steps < 500:
            # El agente elige la MEJOR acción (sin aleatoriedad)
            action = agent.act(state, training=False)
            next_scan, reward, done = env.step(action)
            state = state_proc.get_state(next_scan, env.position, env.goal_position, env.yaw)
            steps += 1
            
            # Verificar si fue éxito real (recompensa alta de meta)
            if reward > 100: 
                # Nota: No sumamos success_count aquí todavía, esperamos a validar los pasos
                pass

        # --- FILTRO DE SALIDA EN FALSO ---
        # Si el episodio duró menos de 5 pasos, fue un error de spawn.
        # Lo ignoramos y NO aumentamos el contador 'current_episode'.
        if steps < 5:
            continue 

        # Si llegamos aquí, el intento fue válido
        current_episode += 1
        
        # Determinar resultado
        if reward > 100:
            outcome = "GANÓ 🏆"
            success_count += 1
        else:
            outcome = "Perdió ❌"
            
        print(f"Test {current_episode}/{total_episodes}: {outcome} (Pasos: {steps})")

    # 4. Resultado Final
    success_rate = (success_count / total_episodes) * 100
    print("\n" + "="*40)
    print(f"RESULTADO FINAL: {success_rate:.1f}% de Éxito")
    print("="*40)
    
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()