import heapq

class SpaceTimeAStar:
    def __init__(self, grid_width=20, grid_height=24):
        self.grid_width = grid_width
        self.grid_height = grid_height
        
        # Lanes configuration
        self.vertical_lanes = [8, 9, 10, 11]
        self.horizontal_lanes = [10, 11, 12, 13]
        
    def is_on_road(self, x, y):
        # Check if coordinates are on the vertical or horizontal road
        is_vert = (x in self.vertical_lanes) and (0 <= y < self.grid_height)
        is_horiz = (y in self.horizontal_lanes) and (0 <= x < self.grid_width)
        return is_vert or is_horiz

    def get_neighbors(self, x, y):
        neighbors = []
        # 1. Wait action is always a candidate
        neighbors.append((x, y))
        
        # 2. Moving actions
        # Standard lane-based moves
        # Eastbound: rows 10, 11
        if y in [10, 11]:
            neighbors.append((x + 1, y))
        # Westbound: rows 12, 13
        if y in [12, 13]:
            neighbors.append((x - 1, y))
        # Northbound: cols 10, 11
        if x in [10, 11]:
            neighbors.append((x, y + 1))
        # Southbound: cols 8, 9
        if x in [8, 9]:
            neighbors.append((x, y - 1))
            
        # 3. Turns at the intersection
        in_intersection = (8 <= x <= 11) and (10 <= y <= 13)
        if in_intersection:
            # Add turns: from vertical to horizontal and vice versa
            # If on vertical southbound (8, 9), can turn to horizontal (10, 11, 12, 13)
            if x in [8, 9]:
                neighbors.append((x - 1, y)) # Turn West
                neighbors.append((x + 1, y)) # Turn East
            # If on vertical northbound (10, 11), can turn to horizontal
            elif x in [10, 11]:
                neighbors.append((x - 1, y)) # Turn West
                neighbors.append((x + 1, y)) # Turn East
            
            # If on horizontal eastbound (10, 11), can turn to vertical
            if y in [10, 11]:
                neighbors.append((x, y - 1)) # Turn South
                neighbors.append((x, y + 1)) # Turn North
            # If on horizontal westbound (12, 13), can turn to vertical
            elif y in [12, 13]:
                neighbors.append((x, y - 1)) # Turn South
                neighbors.append((x, y + 1)) # Turn North

        # 4. Lane changes (same road, adjacent lanes)
        if y == 10: neighbors.append((x, 11))
        elif y == 11: neighbors.append((x, 10))
        elif y == 12: neighbors.append((x, 13))
        elif y == 13: neighbors.append((x, 12))
        
        if x == 8: neighbors.append((9, y))
        elif x == 9: neighbors.append((8, y))
        elif x == 10: neighbors.append((11, y))
        elif x == 11: neighbors.append((10, y))

        # Filter out of bounds and off road
        valid_neighbors = []
        for nx, ny in neighbors:
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                if self.is_on_road(nx, ny):
                    valid_neighbors.append((nx, ny))
                    
        # Remove duplicates
        return list(set(valid_neighbors))

    def heuristic(self, x, y, goal):
        # Manhattan distance to the goal
        return abs(x - goal[0]) + abs(y - goal[1])

    def find_path(self, start, goal, vertex_constraints, edge_constraints, dynamic_obstacles=None, max_t=100):
        # start, goal are (x, y) tuples
        # vertex_constraints is a set of (x, y, t)
        # edge_constraints is a set of (x1, y1, x2, y2, t)
        # dynamic_obstacles is a set of (x, y, t) for pedestrians
        
        if dynamic_obstacles is None:
            dynamic_obstacles = set()
            
        # Priority Queue elements: (f_score, g_score, x, y, t, parent)
        # parent is a reference to the parent tuple
        start_state = (self.heuristic(start[0], start[1], goal), 0, start[0], start[1], 0, None)
        open_set = [start_state]
        
        # Closed set stores (x, y, t)
        closed_set = set()
        
        while open_set:
            f, g, x, y, t, parent = heapq.heappop(open_set)
            
            state_key = (x, y, t)
            if state_key in closed_set:
                continue
            closed_set.add(state_key)
            
            # Check goal condition
            # In time-space planning, we are at the goal if (x, y) == goal
            # AND there are no future constraints on this goal cell.
            if (x, y) == goal:
                # Check if there are any future vertex constraints at the goal
                has_future_constraint = False
                for future_t in range(t + 1, max_t):
                    if (x, y, future_t) in vertex_constraints:
                        has_future_constraint = True
                        break
                if not has_future_constraint:
                    # Reconstruct path
                    path = []
                    curr = (f, g, x, y, t, parent)
                    while curr is not None:
                        path.append((curr[2], curr[3]))
                        curr = curr[5]
                    path.reverse()
                    return path
            
            if t >= max_t:
                continue
                
            for nx, ny in self.get_neighbors(x, y):
                nt = t + 1
                
                # Check vertex constraints
                if (nx, ny, nt) in vertex_constraints:
                    continue
                    
                # Check edge constraints
                if (x, y, nx, ny, t) in edge_constraints:
                    continue
                    
                # Check dynamic obstacles (pedestrians)
                if (nx, ny, nt) in dynamic_obstacles:
                    continue
                    
                # Calculate scores
                ng = g + 1
                nf = ng + self.heuristic(nx, ny, goal)
                
                heapq.heappush(open_set, (nf, ng, nx, ny, nt, (f, g, x, y, t, parent)))
                
        return None # No path found
