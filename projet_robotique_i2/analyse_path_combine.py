#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
import math
import random
from PIL import Image

class PathPlannerComparatif:
    def __init__(self, map_path, resolution, origin):
        self.resolution = resolution
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        
        # Chargement de la carte PGM
        image = Image.open(map_path)
        self.grid_map = np.array(image)
        self.height, self.width = self.grid_map.shape
        
        # Binarisation : < 200 est considéré comme obstacle ou inconnu
        self.obstacles = self.grid_map < 200

        # Limites de la carte en mètres pour la génération aléatoire
        self.x_min = self.origin_x
        self.x_max = self.origin_x + (self.width * self.resolution)
        self.y_min = self.origin_y
        self.y_max = self.origin_y + (self.height * self.resolution)

    def coord_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = self.height - 1 - int((y - self.origin_y) / self.resolution)
        return (row, col)

    def grid_to_coord(self, row, col):
        x = self.origin_x + (col * self.resolution)
        y = self.origin_y + ((self.height - 1 - row) * self.resolution)
        return (x, y)

    def get_random_coord(self):
        """Génère des coordonnées (x, y) aléatoires dans les limites de la carte."""
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        return (x, y)

    def get_neighbors(self, node):
        r, c = node
        neighbors = []
        # 8 directions avec leurs coûts respectifs (1 pour orthogonal, 1.414 pour diagonal)
        directions = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                      (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        
        for dr, dc, cost in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.height and 0 <= nc < self.width:
                if not self.obstacles[nr, nc]:
                    neighbors.append(((nr, nc), cost))
        return neighbors

    def planifier(self, start_m, goal_m, algorithme):
        start = self.coord_to_grid(start_m[0], start_m[1])
        goal = self.coord_to_grid(goal_m[0], goal_m[1])
        
        # Vérification anti-mur
        if not (0 <= start[0] < self.height and 0 <= start[1] < self.width) or \
           not (0 <= goal[0] < self.height and 0 <= goal[1] < self.width) or \
           self.obstacles[start[0], start[1]] or self.obstacles[goal[0], goal[1]]:
            return None, None, None

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        nodes_explored = 0
        
        t0 = time.time()
        
        while open_set:
            current_prio, current = heapq.heappop(open_set)
            nodes_explored += 1
            
            if current == goal:
                break
                
            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    
                    h = math.dist(neighbor, goal) 
                    
                    if algorithme == "astar":
                        f_score = tentative_g_score + h  # f = g + h
                    elif algorithme == "dijkstra":
                        f_score = tentative_g_score      # f = g
                    elif algorithme == "greedy":
                        f_score = h                      # f = h
                        
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        t1 = time.time()
        temps_execution = t1 - t0

        if goal not in came_from and start != goal:
            return None, None, None # Aucun chemin trouvé

        # Reconstruction du chemin
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.append(start)
        path.reverse()

        longueur_chemin = g_score[goal] * self.resolution
        path_coords = [self.grid_to_coord(r, c) for r, c in path]
        
        return path_coords, list(came_from.keys()), (temps_execution, nodes_explored, longueur_chemin)

    def afficher_comparaison(self, start, goal, resultats, num_test):
        plt.figure(figsize=(12, 10))
        plt.imshow(self.grid_map, cmap='gray')
        
        couleurs = {"astar": "red", "dijkstra": "blue", "greedy": "green"}
        styles = {"astar": "-", "dijkstra": "--", "greedy": ":"}
        
        # Affichage des chemins
        for algo, (chemin, explores, stats) in resultats.items():
            if chemin:
                grids = [self.coord_to_grid(x, y) for x, y in chemin]
                path_r, path_c = zip(*grids)
                
                t_exec, noeuds, dist = stats
                label = f"{algo.upper()} (Dist: {dist:.2f}m | Nœuds: {noeuds} | Tps: {t_exec:.3f}s)"
                
                plt.plot(path_c, path_r, color=couleurs[algo], linestyle=styles[algo], linewidth=2.5, label=label)

        # Départ et Arrivée
        s_r, s_c = self.coord_to_grid(start[0], start[1])
        g_r, g_c = self.coord_to_grid(goal[0], goal[1])
        plt.scatter(s_c, s_r, c='green', s=120, marker='o', label='Départ', zorder=5)
        plt.scatter(g_c, g_r, c='purple', s=120, marker='X', label='Arrivée', zorder=5)

        plt.title(f"Test {num_test}/5 : Comparaison A* / Dijkstra / Greedy Best-First", fontsize=14)
        plt.legend(loc="upper right", bbox_to_anchor=(1.0, -0.05), fancybox=True, shadow=True)
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    # --- CONFIGURATION (adapte le chemin si nécessaire) ---
    CHEMIN_MAP = '/home/benbruno/turtlebot3_ws/src/projet_robotique_i2/maps/map_world.pgm'
    RESOLUTION = 0.05
    ORIGIN = [-10.3, -10.7]
    
    planificateur = PathPlannerComparatif(CHEMIN_MAP, RESOLUTION, ORIGIN)
    algorithmes = ["astar", "dijkstra", "greedy"]
    
    print("\nLancement de la série de 5 tests aléatoires...")
    
    for i in range(1, 6):
        print(f"\n{'='*40}")
        print(f"TEST {i}/5")
        
        # Génération aléatoire
        start = planificateur.get_random_coord()
        goal = planificateur.get_random_coord()
        
        print(f"Départ : ({start[0]:.2f}, {start[1]:.2f}) | Arrivée : ({goal[0]:.2f}, {goal[1]:.2f})")
        
        resultats_test = {}
        chemin_possible = True
        
        # Exécution des 3 algorithmes
        for algo in algorithmes:
            chemin, explores, stats = planificateur.planifier(start, goal, algo)
            
            if chemin is None:
                chemin_possible = False
                break # Inutile de tester les autres si le premier échoue (mur)
            else:
                resultats_test[algo] = (chemin, explores, stats)
                
        if not chemin_possible:
            print("CHEMIN IMPOSSIBLE (Départ/Arrivée sur un mur ou inatteignable).")
        else:
            print("Chemins trouvés ! Affichage du graphique...")
            planificateur.afficher_comparaison(start, goal, resultats_test, i)