#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque
import time

# Import de l'environnement que nous avons créé
from robot_env import TurtleBotEnv

# --- 1. LE RÉSEAU DE NEURONES (DQN) ---
class DQN(nn.Module):
    def __init__(self, input_size, output_size):
        super(DQN, self).__init__()
        # Architecture simple : 3 couches de neurones
        self.fc1 = nn.Linear(input_size, 64)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, output_size)

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)

# --- 2. LA MÉMOIRE (REPLAY BUFFER) ---
class ReplayBuffer:
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return np.array(states), actions, rewards, np.array(next_states), dones

    def __len__(self):
        return len(self.buffer)

# --- 3. L'AGENT ET LA BOUCLE D'ENTRAÎNEMENT ---
def train():
    rclpy.init()
    
    # Coordonnées de la cible (à modifier selon ton test)
    env = TurtleBotEnv(goal_x=-5.43, goal_y=-2.26)
    
    # Hyperparamètres
    STATE_SIZE = 26  # 24 LiDAR + 1 distance + 1 angle
    ACTION_SIZE = 5  # Les 5 commandes de vitesse
    BATCH_SIZE = 64
    GAMMA = 0.99     # Facteur de discount (importance des récompenses futures)
    EPSILON = 1.0    # Taux d'exploration initial (100% aléatoire)
    EPSILON_MIN = 0.05
    EPSILON_DECAY = 0.995
    LR = 0.001       # Learning rate
    EPISODES = 100   # Nombre d'essais pour apprendre
    
    # Initialisation du modèle PyTorch et de l'optimiseur
    policy_net = DQN(STATE_SIZE, ACTION_SIZE)
    optimizer = optim.Adam(policy_net.parameters(), lr=LR)
    loss_fn = nn.MSELoss()
    memory = ReplayBuffer(10000)
    
    print("🚀 Début de l'entraînement DQN...")

    for episode in range(EPISODES):
        # Laisser ROS 2 traiter les messages entrants
        rclpy.spin_once(env, timeout_sec=0.1)
        
        state = env.reset()
        total_reward = 0
        done = False
        step = 0
        
        while not done and step < 500: # Max 500 étapes par épisode
            rclpy.spin_once(env, timeout_sec=0.1) # Mise à jour des capteurs
            
            # --- CHOIX DE L'ACTION (Epsilon-Greedy) ---
            if random.random() <= EPSILON:
                action = random.randrange(ACTION_SIZE) # Exploration : action au hasard
            else:
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                with torch.no_grad():
                    q_values = policy_net(state_tensor)
                action = torch.argmax(q_values).item() # Exploitation : meilleure action selon l'IA
                
            # --- EXÉCUTION DE L'ACTION ---
            next_state, reward, done = env.step(action)
            total_reward += reward
            
            # --- STOCKAGE EN MÉMOIRE ---
            memory.push(state, action, reward, next_state, done)
            state = next_state
            step += 1
            
            # --- OPTIMISATION DU RÉSEAU DE NEURONES ---
            if len(memory) > BATCH_SIZE:
                # 1. Tirer un lot d'expériences au hasard
                states, actions, rewards, next_states, dones = memory.sample(BATCH_SIZE)
                
                states_t = torch.FloatTensor(states)
                actions_t = torch.LongTensor(actions).unsqueeze(1)
                rewards_t = torch.FloatTensor(rewards).unsqueeze(1)
                next_states_t = torch.FloatTensor(next_states)
                dones_t = torch.FloatTensor(dones).unsqueeze(1)
                
                # 2. Calculer les Q-Values actuelles
                q_values = policy_net(states_t).gather(1, actions_t)
                
                # 3. Calculer les Q-Values cibles (l'objectif à atteindre)
                with torch.no_grad():
                    max_next_q_values = policy_net(next_states_t).max(1)[0].unsqueeze(1)
                    target_q_values = rewards_t + GAMMA * max_next_q_values * (1 - dones_t)
                
                # 4. Rétropropagation (Mise à jour des poids du cerveau)
                loss = loss_fn(q_values, target_q_values)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        # Diminuer le taux d'exploration
        if EPSILON > EPSILON_MIN:
            EPSILON *= EPSILON_DECAY
            
        print(f"Épisode {episode+1}/{EPISODES} | Score: {total_reward:.2f} | Epsilon: {EPSILON:.2f}")

    # Sauvegarder le modèle entraîné
    torch.save(policy_net.state_dict(), "dqn_turtlebot.pth")
    print("Entraînement terminé et modèle sauvegardé !")
    
    env.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    train()