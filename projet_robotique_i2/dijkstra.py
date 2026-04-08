import numpy as np
import matplotlib.pyplot as plt
import heapq
from PIL import Image
import sys

def load_map(filepath):
    # Charge l'image pgm et la convertit en grille d'occupation (0 = libre, 1 = obstacle)
    img = Image.open(filepath)
    data = np.array(img)
    # Les pixels clairs (>200) sont libres, les foncés sont des obstacles ou zones inconnues
    grid = np.where(data < 200, 1, 0)
    return grid

def dijkstra(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    oheap = []
    heapq.heappush(oheap, (0, start))
    
    distances_over_time = [] # Pour le graphe

    while oheap:
        current_cost, current = heapq.heappop(oheap)
        
        # Enregistrer la distance de Manhattan au but pour le graphe
        dist_to_goal = abs(current[0] - goal[0]) + abs(current[1] - goal[1])
        distances_over_time.append(dist_to_goal)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], distances_over_time

        close_set.add(current)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            move_cost = 1.414 if i != 0 and j != 0 else 1.0 # Coût diagonal vs orthogonal
            tentative_g_score = gscore[current] + move_cost

            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else: continue
            else: continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                heapq.heappush(oheap, (tentative_g_score, neighbor))
                
    return False, []

if __name__ == '__main__':
    grid = load_map('map_world.pgm')
    start = (29, 81) 
    goal = (231, 362) 
    
    print("Calcul du chemin avec Dijkstra...")
    path, dist_history = dijkstra(grid, start, goal)
    
    if path:
        print(f"Chemin trouvé ! Longueur : {len(path)} étapes.")
        # Affichage de la carte
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.imshow(grid, cmap='Greys')
        y, x = zip(*path)
        ax1.plot(x, y, 'b-', linewidth=2, label='Path')
        ax1.plot(start[1], start[0], 'go', markersize=8, label='Start')
        ax1.plot(goal[1], goal[0], 'ro', markersize=8, label='Goal')
        ax1.legend()
        ax1.set_title("Dijkstra Path")
        
        # Affichage du graphe
        ax2.plot(dist_history)
        ax2.set_xlabel("Itérations (Temps)")
        ax2.set_ylabel("Distance de Manhattan au But")
        ax2.set_title("Évolution de la distance vers le but")
        plt.show()
    else:
        print("Aucun chemin trouvé.")