import heapq


class Node:
    def __init__(self, x, y, cost=0, priority=0):
        self.x = x
        self.y = y
        self.cost = cost
        self.priority = priority
        self.parent = None

    def __lt__(self, other):
        return self.priority < other.priority


# A* pathfinding algorithm
def heuristic(a, b):
    # Manhattan distance heuristic
    return abs(a.x - b.x) + abs(a.y - b.y)  

def a_star(start, goal, grid, conflict_map):
    open_list = []
    heapq.heappush(open_list, start)
    closed_set = set()

    while open_list:
        current_node = heapq.heappop(open_list)

        if (current_node.x, current_node.y) == (goal.x, goal.y):
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        closed_set.add((current_node.x, current_node.y))

        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            nx, ny = current_node.x + dx, current_node.y + dy
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
                if (nx, ny) not in closed_set and grid[nx][ny] == 0 and (nx, ny) not in conflict_map:
                    neighbor = Node(nx, ny)
                    neighbor.cost = current_node.cost + 1
                    neighbor.priority = neighbor.cost + heuristic(neighbor, goal)
                    neighbor.parent = current_node
                    heapq.heappush(open_list, neighbor)

    return None
