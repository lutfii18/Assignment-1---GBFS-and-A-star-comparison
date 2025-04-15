import heapq
import time
import random


def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def neighbors(pos, grid):
    rows, cols = len(grid), len(grid[0])
    x, y = pos
    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != '#':
            yield (nx, ny)


def gbfs(start, goal, grid):
    frontier = [(manhattan(start, goal), start)]
    came_from = {start: None}
    explored = set()
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        explored.add(current)
        for next in neighbors(current, grid):
            if next not in came_from:
                came_from[next] = current
                heapq.heappush(frontier, (manhattan(next, goal), next))
    return reconstruct_path(came_from, start, goal), len(explored)


def astar(start, goal, grid):
    frontier = [(manhattan(start, goal), 0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored = set()
    while frontier:
        _, g, current = heapq.heappop(frontier)
        if current == goal:
            break
        explored.add(current)
        for next in neighbors(current, grid):
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


def generate_grid(n_node, n_obstacle):
    side = int(n_node ** 0.5)
    grid = [['.' for _ in range(side)] for _ in range(side)]
    count = 0
    while count < n_obstacle:
        x, y = random.randint(0, side-1), random.randint(0, side-1)
        if grid[x][y] == '.':
            grid[x][y] = '#'
            count += 1
    while True:
        sx, sy = random.randint(0, side-1), random.randint(0, side-1)
        gx, gy = random.randint(0, side-1), random.randint(0, side-1)
        if (sx, sy) != (gx, gy) and grid[sx][sy] == '.' and grid[gx][gy] == '.':
            grid[sx][sy] = 'S'
            grid[gx][gy] = 'G'
            return grid, (sx, sy), (gx, gy)


def print_grid_with_path(grid, path):
    display = [row.copy() for row in grid]
    for r, c in path:
        if display[r][c] not in 'SG':
            display[r][c] = '*'
    for row in display:
        print(''.join(row))
    print()


experiments = [
    (5000, 10),
    (50000, 100),
    (500000, 1000),
    (5000000, 10000),
    (50000000, 100000),
]

print("Time (ms)")
print("Experiment\tGBFS\t\tA*")

gbfs_times = []
astar_times = []
gbfs_lengths = []
astar_lengths = []

for idx, (n_node, n_obstacle) in enumerate(experiments, 1):
    print(f"\n=== Experiment #{idx} ===")
    print(f"#node: {n_node} | #obstacle: {n_obstacle}")
    
    grid, start, goal = generate_grid(n_node, n_obstacle)

    t0 = time.time()
    gbfs_path, _ = gbfs(start, goal, grid)
    gbfs_time = (time.time() - t0) * 1000
    gbfs_times.append(gbfs_time)
    gbfs_lengths.append(len(gbfs_path))

    t1 = time.time()
    astar_path, _ = astar(start, goal, grid)
    astar_time = (time.time() - t1) * 1000
    astar_times.append(astar_time)
    astar_lengths.append(len(astar_path))

    print("GBFS Path:")
    print_grid_with_path(grid, gbfs_path)

    print("A* Path:")
    print_grid_with_path(grid, astar_path)

    print(f"GBFS time: {gbfs_time:.2f} ms | Path length: {len(gbfs_path)}")
    print(f"A*   time: {astar_time:.2f} ms | Path length: {len(astar_path)}")


print("\n=== Summary ===")
print("Time (ms)")
print(f"{'Experiment':<12}{'GBFS':>12}{'A*':>12}")
for i in range(5):
    print(f"#{i+1:<11}{gbfs_times[i]:>12.2f}{astar_times[i]:>12.2f}")
print(f"{'Average':<12}{sum(gbfs_times)/5:>12.2f}{sum(astar_times)/5:>12.2f}")

print("\nPath Length")
print(f"{'Experiment':<12}{'GBFS':>12}{'A*':>12}")
for i in range(5):
    print(f"#{i+1:<11}{gbfs_lengths[i]:>12}{astar_lengths[i]:>12}")
print(f"{'Average':<12}{sum(gbfs_lengths)//5:>12}{sum(astar_lengths)//5:>12}")

