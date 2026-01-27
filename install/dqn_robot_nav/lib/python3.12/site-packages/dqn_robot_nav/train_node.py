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
    
    print("Esperando conexión con simulador...")
    while env.scan_data is None:
        rclpy.spin_once(env, timeout_sec=0.1)
    
    state_size = 12
    action_size = env.action_size
    agent = DQNAgent(state_size, action_size)
    
    # --- CAMBIO 1: Usamos un contador manual en lugar de un for ---
    max_episodes = 500
    current_episode = 0
    
    print(f"Iniciando entrenamiento... Meta: {max_episodes} episodios validos.")

    while current_episode < max_episodes:
        # Resetear entorno
        scan = env.reset()
        state = state_proc.get_state(scan, env.position, env.goal_position, env.yaw)
        
        total_reward = 0
        steps = 0
        done = False
        
        # Bucle del episodio
        while not done:
            action = agent.act(state)
            next_scan, reward, done = env.step(action)
            next_state = state_proc.get_state(next_scan, env.position, env.goal_position, env.yaw)
            
            agent.remember(state, action, reward, next_state, done)
            loss = agent.replay()
            
            state = next_state
            total_reward += reward
            steps += 1
            
            if steps > 500:
                done = True
        
        # --- CAMBIO 2: FILTRO DE SALIDA EN FALSO ---
        # Si el episodio duró muy poco (menos de 5 pasos), fue un error de spawn.
        if steps < 2:
            #  print(f"⚠️ Falso arranque detectado (Pasos: {steps}). Ignorando episodio...")
            # NO incrementamos current_episode.
            # NO guardamos modelo.
            # El bucle while se repite y volvemos a intentar este mismo número de episodio.
            continue 

        # Si llegamos aquí, el episodio fue válido
        current_episode += 1
        
        print(f"Episodio: {current_episode}/{max_episodes} | Pasos: {steps} | Recompensa: {total_reward:.2f} | Epsilon: {agent.epsilon:.2f}")
                
        # Guardar modelo cada 10 episodios VÁLIDOS
        if current_episode % 10 == 0:
            save_path = os.path.join(save_dir, f"dqn_model_{current_episode}.pkl")
            agent.save(save_path)

    print("Entrenamiento finalizado.")
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()