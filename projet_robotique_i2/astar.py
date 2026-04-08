import numpy as np
import matplotlib.pyplot as plt
import heapq
from PIL import Image

def load_map(filepath):
    img = Image.open(filepath)
    grid = np.where(np.array(img) < 200, 1, 0)
    return grid

def heuristic(a, b):
    # Distance Euclidienne
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
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
            move_cost = 1.414 if i != 0 and j != 0 else 1.0
            tentative_g_score = gscore[current] + move_cost

            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1: continue
            else: continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): continue

            if tentative_g_score < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                
    return False, []

if __name__ == '__main__':
    grid = load_map('map_world.pgm')
    start = (29, 81) 
    goal = (231, 362)  
    
    print("Calcul du chemin avec A*...")
    path, dist_history = astar(grid, start, goal)
    
    if path:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        ax1.imshow(grid, cmap='Greys')
        y, x = zip(*path)
        ax1.plot(x, y, 'g-', linewidth=2, label='Path')
        ax1.plot(start[1], start[0], 'go', markersize=8)
        ax1.plot(goal[1], goal[0], 'ro', markersize=8)
        ax1.set_title("A* Path")
        
        ax2.plot(dist_history, color='g')
        ax2.set_xlabel("Itérations (Temps)")
        ax2.set_ylabel("Distance Euclidienne au But")
        ax2.set_title("Convergence vers le but")
        plt.show()