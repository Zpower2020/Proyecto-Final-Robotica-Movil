import numpy as np
from sklearn.neural_network import MLPRegressor
from collections import deque
import random
import pickle
import os
import copy # Necesario para clonar la red

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        
        # Hiperparámetros
        self.gamma = 0.99    
        self.epsilon = 1.0   
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.99995
        self.batch_size = 64
        self.memory = deque(maxlen=20000)
        
        # Red Neuronal Principal (Policy Network)
        self.model = MLPRegressor(hidden_layer_sizes=(64, 64), activation='relu', solver='adam', learning_rate_init=0.001)
        # Inicialización
        self.model.partial_fit(np.zeros((1, state_size)), np.zeros((1, action_size)))
        
        # --- REQUISITO CUMPLIDO: Target Network ---
        # Creamos una copia exacta del modelo para usarla como objetivo estable
        self.target_model = copy.deepcopy(self.model)
        
        # Contador para actualizar la target network
        self.target_update_counter = 0

    def update_target_model(self):
        """Copia los pesos de la red principal a la red objetivo"""
        self.target_model = copy.deepcopy(self.model)
        # print("Target network updated!")

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state, training=True):
        if training and np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        
        act_values = self.model.predict(state.reshape(1, -1))
        return np.argmax(act_values[0])

    def replay(self):
        if len(self.memory) < self.batch_size:
            return 0.0
            
        minibatch = random.sample(self.memory, self.batch_size)
        
        states = np.array([i[0] for i in minibatch])
        next_states = np.array([i[3] for i in minibatch])
        
        # 1. Predecir con la red PRINCIPAL
        q_current = self.model.predict(states)
        
        # 2. Predecir el futuro con la TARGET NETWORK (Más estabilidad)
        q_next = self.target_model.predict(next_states)
        
        X = []
        y = []
        
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            target = reward
            if not done:
                # Ecuación de Bellman usando la red objetivo
                target = reward + self.gamma * np.amax(q_next[i])
            
            target_f = q_current[i]
            target_f[action] = target
            
            X.append(state)
            y.append(target_f)
            
        self.model.partial_fit(np.array(X), np.array(y))
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
            
        # Actualizar la red objetivo cada 20 pasos de entrenamiento (ajustable)
        self.target_update_counter += 1
        if self.target_update_counter > 20:
            self.update_target_model()
            self.target_update_counter = 0
            
        return self.model.loss_

    def save(self, filepath):
        # Guardamos ambas por seguridad, pero solo necesitamos la principal
        with open(filepath, 'wb') as f:
            pickle.dump(self.model, f)

    def load(self, filepath):
        if os.path.exists(filepath):
            with open(filepath, 'rb') as f:
                self.model = pickle.load(f)
            self.target_model = copy.deepcopy(self.model)
            self.epsilon = 0.05