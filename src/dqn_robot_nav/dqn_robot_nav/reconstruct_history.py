import rclpy
import os
import glob
import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor

def extract_episode_num(filename):
    # Extraer el número del nombre del archivo (dqn_model_150.pkl -> 150)
    match = re.search(r'dqn_model_(\d+).pkl', filename)
    return int(match.group(1)) if match else 0

def main(args=None):
    rclpy.init(args=args)
    
    env = TurtleBot3Env()
    state_proc = StateProcessor(n_lidar_bins=10)
    state_size = 12
    action_size = env.action_size
    agent = DQNAgent(state_size, action_size)
    
    # Buscar todos los modelos guardados
    models_dir = os.path.join(os.getcwd(), 'saved_models')
    model_files = glob.glob(os.path.join(models_dir, "*.pkl"))
    
    # Ordenarlos numéricamente (10, 20, 30...)
    model_files.sort(key=extract_episode_num)
    
    history_data = []
    
    print(f"Encontrados {len(model_files)} modelos. Iniciando reconstrucción histórica...")
    print("Esto tomará unos minutos")

    for model_path in model_files:
        episode_num = extract_episode_num(model_path)
        
        # Cargar el cerebro de ese momento
        agent.load(model_path)
        agent.epsilon = 0.0 # Evaluación pura, sin ruido
        
        # Hacer 3 pruebas rápidas para sacar promedio
        rewards = []
        for _ in range(3):
            scan = env.reset()
            state = state_proc.get_state(scan, env.position, env.goal_position, env.yaw)
            total_reward = 0
            done = False
            steps = 0
            
            while not done and steps < 300: # Límite corto para ir rápido
                action = agent.act(state, training=False)
                next_scan, reward, done = env.step(action)
                state = state_proc.get_state(next_scan, env.position, env.goal_position, env.yaw)
                total_reward += reward
                steps += 1
            
            rewards.append(total_reward)
        
        avg_reward = np.mean(rewards)
        print(f"Modelo Episodio {episode_num}: Recompensa Promedio = {avg_reward:.2f}")
        
        history_data.append({
            'Episode': episode_num,
            'Avg_Reward': avg_reward
        })

    # Generar Gráfica
    df = pd.DataFrame(history_data)
    
    plt.figure(figsize=(10, 6))
    plt.plot(df['Episode'], df['Avg_Reward'], marker='o', linestyle='-', color='b')
    plt.title('Curva del entrenamiento')
    plt.xlabel('Episodios de Entrenamiento')
    plt.ylabel('Recompensa Promedio (Evaluación)')
    plt.grid(True)
    plt.savefig('grafica_curva.png')
    
    print("¡Listo! Gráfica guardada como 'grafica_curva.png'")
    
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()