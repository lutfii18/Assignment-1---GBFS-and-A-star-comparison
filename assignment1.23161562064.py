import heapq
import time

grid_str = [
    "S..#......",
    ".#.#.####.",
    ".#......#.",
    ".#####..#.",
    ".....#..#G",
    "####.#..##",
    "...#.#....",
    ".#.#.####.",
    ".#........",
    "....#####."
]

grid = [list(row) for row in grid_str]

# Ukuran grid
rows, cols = len(grid), len(grid[0])

for i in range(rows):
    for j in range(cols):
        if grid[i][j] == 'S':
            start = (i, j)
        elif grid[i][j] == 'G':
            goal = (i, j)

def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def neighbors(pos):
    x, y = pos
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != '#':
            yield (nx, ny)

def gbfs(start, goal):
    frontier = [(manhattan(start, goal), start)]
    came_from = {start: None}
    explored = set()
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        explored.add(current)
        for next in neighbors(current):
            if next not in came_from:
                came_from[next] = current
                heapq.heappush(frontier, (manhattan(next, goal), next))
    return reconstruct_path(came_from, start, goal), len(explored)

def astar(start, goal):
    frontier = [(manhattan(start, goal), 0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored = set()
    while frontier:
        _, g, current = heapq.heappop(frontier)
        if current == goal:
            break
        explored.add(current)
        for next in neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + manhattan(next, goal)
                heapq.heappush(frontier, (priority, new_cost, next))
                came_from[next] = current
    return reconstruct_path(came_from, start, goal), len(explored)

def reconstruct_path(came_from, start, goal):
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return []  
    path.append(start)
    path.reverse()
    return path

def print_path(path):
    grid_copy = [list(row) for row in grid_str]
    for r, c in path:
        if grid_copy[r][c] not in 'SG':
            grid_copy[r][c] = '*'
    for row in grid_copy:
        print("".join(row))

start_time = time.time()
gbfs_path, gbfs_explored = gbfs(start, goal)
gbfs_time = time.time() - start_time

start_time = time.time()
astar_path, astar_explored = astar(start, goal)
astar_time = time.time() - start_time

print("=== GBFS ===")
print_path(gbfs_path)
print(f"Length: {len(gbfs_path)}")
print(f"Nodes explored: {gbfs_explored}")
print(f"Time: {gbfs_time:.6f}s")

print("\n=== A* ===")
print_path(astar_path)
print(f"Length: {len(astar_path)}")
print(f"Nodes explored: {astar_explored}")
print(f"Time: {astar_time:.6f}s")
