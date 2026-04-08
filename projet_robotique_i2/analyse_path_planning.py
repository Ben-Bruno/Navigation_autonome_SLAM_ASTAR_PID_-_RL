#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time
from PIL import Image
import math 


class PathPlanner:
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

    def coord_to_grid(self, x, y):
        """Convertit les coordonnées réelles (m) en indices de matrice"""
        col = int((x - self.origin_x) / self.resolution)
        row = self.height - 1 - int((y - self.origin_y) / self.resolution)
        return (row, col)

    def grid_to_coord(self, row, col):
        """Convertit les indices de matrice en coordonnées réelles (m)"""
        x = self.origin_x + (col * self.resolution)
        y = self.origin_y + ((self.height - 1 - row) * self.resolution)
        return (x, y)

    def get_neighbors(self, node):
        r, c = node
        neighbors = []
        # Mouvements dans les 8 directions
        directions = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                      (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        
        for dr, dc, cost in directions:
            nr, nc = r + dr, c + dc
            # Vérification des limites et des obstacles
            if 0 <= nr < self.height and 0 <= nc < self.width:
                if not self.obstacles[nr, nc]:
                    neighbors.append(((nr, nc), cost))
        return neighbors

    def planifier(self, start_m, goal_m, algorithme="astar"):
        """Exécute l'algorithme choisi et retourne les métriques."""
        start = self.coord_to_grid(start_m[0], start_m[1])
        goal = self.coord_to_grid(goal_m[0], goal_m[1])
        
        if self.obstacles[start[0], start[1]] or self.obstacles[goal[0], goal[1]]:
            print("Erreur : Le départ ou l'arrivée est dans un obstacle.")
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
                    
                    # --- C'EST ICI QUE LES ALGORITHMES DIFFÈRENT ---
                    # Heuristique (Distance Euclidienne)
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

        # Reconstruction du chemin
        path = []
        if goal in came_from:
            curr = goal
            while curr != start:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()

        longueur_chemin = g_score.get(goal, float('inf')) * self.resolution
        
        # Conversion du chemin en coordonnées réelles
        path_coords = [self.grid_to_coord(r, c) for r, c in path]
        
        print(f"--- Résultats pour {algorithme.upper()} ---")
        print(f"Temps d'exécution : {temps_execution:.4f} secondes")
        print(f"Nœuds explorés (Coût computationnel) : {nodes_explored}")
        print(f"Longueur du chemin (Optimalité) : {longueur_chemin:.2f} mètres\n")
        
        return path_coords, list(came_from.keys()), (temps_execution, nodes_explored, longueur_chemin)

    def afficher_resultats(self, path, explored, start, goal, title):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.grid_map, cmap='gray')
        
        # Afficher la zone explorée (les nœuds évalués par l'algorithme)
        if explored:
            ex_r, ex_c = zip(*explored)
            plt.scatter(ex_c, ex_r, c='cyan', s=1, alpha=0.3, label='Nœuds explorés')

        # Afficher le chemin géométrique calculé
        if path:
            grids = [self.coord_to_grid(x, y) for x, y in path]
            path_r, path_c = zip(*grids)
            plt.plot(path_c, path_r, c='red', linewidth=2, label='Chemin (Path)')

        # Départ et Arrivée
        s_r, s_c = self.coord_to_grid(start[0], start[1])
        g_r, g_c = self.coord_to_grid(goal[0], goal[1])
        plt.scatter(s_c, s_r, c='green', s=100, marker='o', label='Départ')
        plt.scatter(g_c, g_r, c='blue', s=100, marker='X', label='Arrivée')

        plt.title(title)
        plt.legend()
        plt.show()

if __name__ == "__main__":
    
    CHEMIN_MAP = '/home/benbruno/turtlebot3_ws/src/projet_robotique_i2/maps/map_world.pgm'
    RESOLUTION = 0.05
    ORIGIN = [-10.3, -10.7]
    
    planificateur = PathPlanner(CHEMIN_MAP, RESOLUTION, ORIGIN)
    
    DEPART = (-9.06, 5.93)
    ARRIVEE = (-0.86, 5.86)

    
    METHODE = "astar" 
    
    chemin, explores, stats = planificateur.planifier(DEPART, ARRIVEE, algorithme=METHODE)
    print("Voici le chemin à enprunter: ")
    print(chemin)
    
    if chemin:
        planificateur.afficher_resultats(chemin, explores, DEPART, ARRIVEE, f"Path Planning: {METHODE.upper()}")