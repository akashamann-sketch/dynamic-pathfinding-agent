import heapq
import time
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =========================
# 1) Grid Setup
# =========================
ROWS, COLS = 20, 20
DENSITY = 0.30

START = (0, 0)
GOAL  = (ROWS - 1, COLS - 1)

def make_grid(rows, cols, density):
    # 0 = empty, 1 = wall
    grid = [[0 for _ in range(cols)] for _ in range(rows)]

    for r in range(rows):
        for c in range(cols):
            if (r, c) in (START, GOAL):
                continue
            if random.random() < density:
                grid[r][c] = 1

    return grid

grid = make_grid(ROWS, COLS, DENSITY)

# =========================
# 2) Heuristics
# =========================
def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# Choose heuristic (comment/uncomment)
H = manhattan
# H = euclidean


# =========================
# 3) Neighbors (4-dir)
# =========================
def neighbors(node):
    r, c = node
    for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
        nr, nc = r + dr, c + dc

        if 0 <= nr < ROWS and 0 <= nc < COLS:
            if grid[nr][nc] == 0:
                yield (nr, nc)


# ==========================================================
# 4) A* Search (clear student style)
# ==========================================================
def astar(start, goal):
    pq = []
    heapq.heappush(pq, (0, start))

    g_cost = {start: 0}
    parent = {start: None}

    visited = set()
    frontier = {start}

    t0 = time.perf_counter()

    while pq:
        f, node = heapq.heappop(pq)

        if node in visited:
            continue

        frontier.discard(node)
        visited.add(node)

        if node == goal:
            break

        for nb in neighbors(node):
            new_g = g_cost[node] + 1

            if nb not in g_cost or new_g < g_cost[nb]:
                g_cost[nb] = new_g
                f_cost = new_g + H(nb, goal)
                heapq.heappush(pq, (f_cost, nb))
                parent[nb] = node
                frontier.add(nb)

    t1 = time.perf_counter()

    if goal not in parent:
        return [], visited, frontier, (t1 - t0) * 1000

    # path reconstruction
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()

    return path, visited, frontier, (t1 - t0) * 1000


# ==========================================================
# 5) GBFS Search (clear student style)
# ==========================================================
def gbfs(start, goal):
    pq = []
    heapq.heappush(pq, (H(start, goal), start))

    parent = {start: None}

    visited = set()
    frontier = {start}

    t0 = time.perf_counter()

    while pq:
        h, node = heapq.heappop(pq)

        if node in visited:
            continue

        frontier.discard(node)
        visited.add(node)

        if node == goal:
            break

        for nb in neighbors(node):
            # GBFS: first time discovered is kept
            if nb not in parent:
                parent[nb] = node
                heapq.heappush(pq, (H(nb, goal), nb))
                frontier.add(nb)

    t1 = time.perf_counter()

    if goal not in parent:
        return [], visited, frontier, (t1 - t0) * 1000

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()

    return path, visited, frontier, (t1 - t0) * 1000


# =========================
# 6) Matplotlib Visualization
# =========================
algo_name = "A*"
path = []
visited = set()
frontier = set()
time_ms = 0.0
running = False

fig, ax = plt.subplots()
ax.set_xticks([])
ax.set_yticks([])

# display codes:
# 0 empty, 1 wall, 2 visited, 3 frontier, 4 path, 5 start, 6 goal
def build_display():
    display = [[0 for _ in range(COLS)] for _ in range(ROWS)]

    # walls
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 1:
                display[r][c] = 1

    # visited / frontier / path
    for r, c in visited:
        display[r][c] = 2
    for r, c in frontier:
        display[r][c] = 3
    for r, c in path:
        display[r][c] = 4

    # start and goal override
    sr, sc = START
    gr, gc = GOAL
    display[sr][sc] = 5
    display[gr][gc] = 6

    return display

# colors: empty, wall, visited, frontier, path, start, goal
cmap = plt.cm.get_cmap("tab10", 7)
img = ax.imshow(build_display(), vmin=0, vmax=6, cmap=cmap)

def update_title():
    cost = max(0, len(path) - 1)
    ax.set_title(
        f"{algo_name} | Visited: {len(visited)} | Path Cost: {cost} | Time: {time_ms:.2f} ms\n"
        "Click to toggle walls | a=A* | g=GBFS | r=reset"
    )

update_title()

def run_search(name):
    global algo_name, path, visited, frontier, time_ms, running

    algo_name = name

    if name == "A*":
        path, visited, frontier, time_ms = astar(START, GOAL)
    else:
        path, visited, frontier, time_ms = gbfs(START, GOAL)

    running = True
    update_title()

def on_key(event):
    global grid, path, visited, frontier, time_ms, running

    if event.key == "a":
        run_search("A*")

    elif event.key == "g":
        run_search("GBFS")

    elif event.key == "r":
        grid = make_grid(ROWS, COLS, DENSITY)
        grid[START[0]][START[1]] = 0
        grid[GOAL[0]][GOAL[1]] = 0

        path = []
        visited = set()
        frontier = set()
        time_ms = 0.0
        running = False
        update_title()

def on_click(event):
    if running:
        return
    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    c = int(round(event.xdata))
    r = int(round(event.ydata))

    if 0 <= r < ROWS and 0 <= c < COLS:
        if (r, c) not in (START, GOAL):
            grid[r][c] = 1 - grid[r][c]  # toggle wall

fig.canvas.mpl_connect("key_press_event", on_key)
fig.canvas.mpl_connect("button_press_event", on_click)

def animate(_):
    img.set_data(build_display())
    return (img,)

ani = FuncAnimation(fig, animate, interval=80)
plt.show()