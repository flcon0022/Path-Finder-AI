import tkinter as tk
import heapq

# Define the grid size
GRID_WIDTH = 20
GRID_HEIGHT = 20
CELL_SIZE = 30

# Define colors
START_COLOR = 'green'
END_COLOR = 'red'
OBSTACLE_COLOR = 'black'
PATH_COLOR = 'blue'
DEFAULT_COLOR = 'white'
EXPLORED_COLOR = 'yellow'

# Directions for moving: up, down, left, right
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# A* Algorithm Application
class AStarApp:
    def __init__(self, root):
        self.root = root
        self.root.title("A* Algorithm Pathfinding")

        # Create the canvas for the grid
        self.canvas = tk.Canvas(self.root, width=GRID_WIDTH * CELL_SIZE, height=GRID_HEIGHT * CELL_SIZE)
        self.canvas.pack(side=tk.LEFT)

        # Create the canvas for the graph visualization
        self.graph_canvas = tk.Canvas(self.root, width=300, height=300)
        self.graph_canvas.pack(side=tk.RIGHT)

        # Initialize grid
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]  # 0 = empty, 1 = obstacle
        self.start = None
        self.end = None
        self.path = []

        # Create buttons for starting and clearing the algorithm
        self.start_button = tk.Button(self.root, text="Start", command=self.start_algorithm)
        self.start_button.pack(side=tk.LEFT)

        self.clear_button = tk.Button(self.root, text="Clear", command=self.clear_grid)
        self.clear_button.pack(side=tk.LEFT)

        # Bind mouse clicks to grid
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Button-3>", self.on_right_click)

        # Draw initial grid
        self.initialize_grid()

    def initialize_grid(self):
        """Fill the grid with default color."""
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                self.draw_cell(x, y, DEFAULT_COLOR)

    def on_click(self, event):
        """Handles left-clicks for setting start, end, and obstacles."""
        x, y = event.x // CELL_SIZE, event.y // CELL_SIZE
        if self.grid[y][x] == 1:  # Prevent placing start/end on obstacles
            return

        if not self.start:
            self.start = (x, y)
            self.draw_cell(x, y, START_COLOR)
        elif not self.end:
            self.end = (x, y)
            self.draw_cell(x, y, END_COLOR)
        else:
            self.grid[y][x] = 1  # Place an obstacle
            self.draw_cell(x, y, OBSTACLE_COLOR)

    def on_right_click(self, event):
        """Handles right-clicks for removing obstacles."""
        x, y = event.x // CELL_SIZE, event.y // CELL_SIZE
        if (x, y) == self.start or (x, y) == self.end:
            return
        self.grid[y][x] = 0  # Remove the obstacle
        self.draw_cell(x, y, DEFAULT_COLOR)

    def draw_cell(self, x, y, color):
        """Draws a single cell with the given color."""
        self.canvas.create_rectangle(x * CELL_SIZE, y * CELL_SIZE,
                                     (x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE,
                                     fill=color, outline="gray")

    def draw_graph(self, open_nodes, explored_nodes):
        """Draws the graph visualization."""
        self.graph_canvas.delete("all")  # Clear previous graph

        # Draw open nodes
        for node in open_nodes:
            x, y = node
            self.graph_canvas.create_oval(x * 5 + 10, y * 5 + 10, x * 5 + 20, y * 5 + 20, fill='orange')

        # Draw explored nodes
        for node in explored_nodes:
            x, y = node
            self.graph_canvas.create_oval(x * 5 + 10, y * 5 + 10, x * 5 + 20, y * 5 + 20, fill=EXPLORED_COLOR)

    def start_algorithm(self):
        """Starts the A* algorithm."""
        if not self.start or not self.end:
            return

        self.clear_path()
        self.a_star(self.start, self.end)

    def clear_grid(self):
        """Resets the grid and UI."""
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.start = None
        self.end = None
        self.path = []

        # Clear the canvas
        self.canvas.delete("all")
        self.initialize_grid()

        # Clear the graph canvas
        self.graph_canvas.delete("all")

    def clear_path(self):
        """Clears the path without resetting start/end points."""
        for (x, y) in self.path:
            self.draw_cell(x, y, DEFAULT_COLOR)
        self.path = []

    def a_star(self, start, end):
        """A* Algorithm for pathfinding."""
        open_list = []
        closed_list = set()
        came_from = {}

        g_score = {start: 0}
        f_score = {start: self.heuristic(start, end)}

        heapq.heappush(open_list, (f_score[start], start))
        open_nodes = {start}
        explored_nodes = set()

        while open_list:
            _, current = heapq.heappop(open_list)
            open_nodes.discard(current)

            if current == end:
                self.reconstruct_path(came_from, current)
                return

            closed_list.add(current)
            explored_nodes.add(current)

            for direction in DIRECTIONS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if not self.is_valid_cell(neighbor) or neighbor in closed_list:
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, end)

                    if neighbor not in open_nodes:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        open_nodes.add(neighbor)

            self.draw_graph(open_nodes, explored_nodes)
            self.root.update()

        print("No path found!")

    def heuristic(self, current, end):
        """Uses Manhattan distance as heuristic."""
        return abs(current[0] - end[0]) + abs(current[1] - end[1])

    def is_valid_cell(self, cell):
        """Checks if the cell is valid (inside grid and not an obstacle)."""
        x, y = cell
        return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT and self.grid[y][x] == 0

    def reconstruct_path(self, came_from, current):
        """Reconstructs and visualizes the shortest path."""
        self.path.clear()
        while current in came_from:
            current = came_from[current]
            if current != self.start:  # Keep start and end colors
                self.path.append(current)
                self.draw_cell(current[0], current[1], PATH_COLOR)


# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = AStarApp(root)
    root.mainloop()
import tkinter as tk
import heapq

# Define the grid size
GRID_WIDTH = 20
GRID_HEIGHT = 20
CELL_SIZE = 30

# Define colors
START_COLOR = 'green'
END_COLOR = 'red'
OBSTACLE_COLOR = 'black'
PATH_COLOR = 'blue'
DEFAULT_COLOR = 'white'
EXPLORED_COLOR = 'yellow'

# Directions for moving: up, down, left, right
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# A* Algorithm Application
class AStarApp:
    def __init__(self, root):
        self.root = root
        self.root.title("A* Algorithm Pathfinding")

        # Create the canvas for the grid
        self.canvas = tk.Canvas(self.root, width=GRID_WIDTH * CELL_SIZE, height=GRID_HEIGHT * CELL_SIZE)
        self.canvas.pack(side=tk.LEFT)

        # Create the canvas for the graph visualization
        self.graph_canvas = tk.Canvas(self.root, width=300, height=300)
        self.graph_canvas.pack(side=tk.RIGHT)

        # Initialize grid
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]  # 0 = empty, 1 = obstacle
        self.start = None
        self.end = None
        self.path = []

        # Create buttons for starting and clearing the algorithm
        self.start_button = tk.Button(self.root, text="Start", command=self.start_algorithm)
        self.start_button.pack(side=tk.LEFT)

        self.clear_button = tk.Button(self.root, text="Clear", command=self.clear_grid)
        self.clear_button.pack(side=tk.LEFT)

        # Bind mouse clicks to grid
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Button-3>", self.on_right_click)

        # Draw initial grid
        self.initialize_grid()

    def initialize_grid(self):
        """Fill the grid with default color."""
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                self.draw_cell(x, y, DEFAULT_COLOR)

    def on_click(self, event):
        """Handles left-clicks for setting start, end, and obstacles."""
        x, y = event.x // CELL_SIZE, event.y // CELL_SIZE
        if self.grid[y][x] == 1:  # Prevent placing start/end on obstacles
            return

        if not self.start:
            self.start = (x, y)
            self.draw_cell(x, y, START_COLOR)
        elif not self.end:
            self.end = (x, y)
            self.draw_cell(x, y, END_COLOR)
        else:
            self.grid[y][x] = 1  # Place an obstacle
            self.draw_cell(x, y, OBSTACLE_COLOR)

    def on_right_click(self, event):
        """Handles right-clicks for removing obstacles."""
        x, y = event.x // CELL_SIZE, event.y // CELL_SIZE
        if (x, y) == self.start or (x, y) == self.end:
            return
        self.grid[y][x] = 0  # Remove the obstacle
        self.draw_cell(x, y, DEFAULT_COLOR)

    def draw_cell(self, x, y, color):
        """Draws a single cell with the given color."""
        self.canvas.create_rectangle(x * CELL_SIZE, y * CELL_SIZE,
                                     (x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE,
                                     fill=color, outline="gray")

    def draw_graph(self, open_nodes, explored_nodes):
        """Draws the graph visualization."""
        self.graph_canvas.delete("all")  # Clear previous graph

        # Draw open nodes
        for node in open_nodes:
            x, y = node
            self.graph_canvas.create_oval(x * 5 + 10, y * 5 + 10, x * 5 + 20, y * 5 + 20, fill='orange')

        # Draw explored nodes
        for node in explored_nodes:
            x, y = node
            self.graph_canvas.create_oval(x * 5 + 10, y * 5 + 10, x * 5 + 20, y * 5 + 20, fill=EXPLORED_COLOR)

    def start_algorithm(self):
        """Starts the A* algorithm."""
        if not self.start or not self.end:
            return

        self.clear_path()
        self.a_star(self.start, self.end)

    def clear_grid(self):
        """Resets the grid and UI."""
        self.grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]
        self.start = None
        self.end = None
        self.path = []

        # Clear the canvas
        self.canvas.delete("all")
        self.initialize_grid()

        # Clear the graph canvas
        self.graph_canvas.delete("all")

    def clear_path(self):
        """Clears the path without resetting start/end points."""
        for (x, y) in self.path:
            self.draw_cell(x, y, DEFAULT_COLOR)
        self.path = []

    def a_star(self, start, end):
        """A* Algorithm for pathfinding."""
        open_list = []
        closed_list = set()
        came_from = {}

        g_score = {start: 0}
        f_score = {start: self.heuristic(start, end)}

        heapq.heappush(open_list, (f_score[start], start))
        open_nodes = {start}
        explored_nodes = set()

        while open_list:
            _, current = heapq.heappop(open_list)
            open_nodes.discard(current)

            if current == end:
                self.reconstruct_path(came_from, current)
                return

            closed_list.add(current)
            explored_nodes.add(current)

            for direction in DIRECTIONS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if not self.is_valid_cell(neighbor) or neighbor in closed_list:
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, end)

                    if neighbor not in open_nodes:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        open_nodes.add(neighbor)

            self.draw_graph(open_nodes, explored_nodes)
            self.root.update()

        print("No path found!")

    def heuristic(self, current, end):
        """Uses Manhattan distance as heuristic."""
        return abs(current[0] - end[0]) + abs(current[1] - end[1])

    def is_valid_cell(self, cell):
        """Checks if the cell is valid (inside grid and not an obstacle)."""
        x, y = cell
        return 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT and self.grid[y][x] == 0

    def reconstruct_path(self, came_from, current):
        """Reconstructs and visualizes the shortest path."""
        self.path.clear()
        while current in came_from:
            current = came_from[current]
            if current != self.start:  # Keep start and end colors
                self.path.append(current)
                self.draw_cell(current[0], current[1], PATH_COLOR)


# Run the application
if __name__ == "__main__":
    root = tk.Tk()
    app = AStarApp(root)
    root.mainloop()
