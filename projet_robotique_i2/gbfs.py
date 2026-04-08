import numpy as np
import matplotlib.pyplot as plt
import heapq
from PIL import Image

def load_map(filepath):
    img = Image.open(filepath)
    return np.where(np.array(img) < 200, 1, 0)

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1]) # Manhattan est souvent utilisé pour GBFS

def gbfs(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    oheap = []
    # GBFS utilise UNIQUEMENT l'heuristique pour ordonner la file
    heapq.heappush(oheap, (heuristic(start, goal), start))
    
    distances_over_time = []

    while oheap:
        current = heapq.heappop(oheap)[1]
        distances_over_time.append(heuristic(current, goal))

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

            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1: continue
            else: continue

            if neighbor in close_set: continue

            # Pour GBFS on ne s'occupe pas de G score, on ajoute juste le voisin s'il n'est pas vu
            if neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                heapq.heappush(oheap, (heuristic(neighbor, goal), neighbor))
                
    return False, []

if __name__ == '__main__':
    grid = load_map('map_world.pgm')
    start = (150, 150) 
    goal = (300, 300)  
    
    print("Calcul du chemin avec Greedy Best-First Search...")
    path, dist_history = gbfs(grid, start, goal)
    
    if path:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.imshow(grid, cmap='Greys')
        y, x = zip(*path)
        ax1.plot(x, y, 'm-', linewidth=2, label='Path')
        ax1.plot(start[1], start[0], 'go', markersize=8)
        ax1.plot(goal[1], goal[0], 'ro', markersize=8)
        ax1.set_title("GBFS Path")
        
        ax2.plot(dist_history, color='m')
        ax2.set_xlabel("Itérations (Temps)")
        ax2.set_ylabel("Distance de Manhattan au But")
        ax2.set_title("Convergence vers le but")
        plt.show()