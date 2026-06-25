import heapq
import time
from algorithms.search_node import CTNode
from algorithms.a_star import SpaceTimeAStar

def dominates(a, b):
    # a dominates b if a is component-wise <= b and strictly < in at least one component
    all_le = all(x <= y + 1e-9 for x, y in zip(a, b))
    any_lt = any(x < y - 1e-9 for x, y in zip(a, b))
    return all_le and any_lt

def get_agent_pos_at_t(path, t):
    if t < len(path):
        return path[t]
    return path[-1]

def find_all_conflicts(paths):
    # Returns a list of all conflicts found
    # Conflict format:
    # - ('vertex', a1, a2, (x, y), t)
    # - ('edge', a1, a2, (x1, y1), (x2, y2), t)
    conflicts = []
    num_agents = len(paths)
    agent_ids = list(paths.keys())
    
    max_path_len = max(len(p) for p in paths.values()) if paths else 0
    
    for t in range(max_path_len + 5): # Check slightly beyond to ensure goal occupancy safety
        # 1. Check vertex conflicts
        positions = {}
        for a in agent_ids:
            pos = get_agent_pos_at_t(paths[a], t)
            if pos in positions:
                conflicts.append(('vertex', positions[pos], a, pos, t))
            else:
                positions[pos] = a
                
        # 2. Check edge conflicts (swapping)
        for i in range(num_agents):
            a1 = agent_ids[i]
            if t < len(paths[a1]) - 1:
                u1 = paths[a1][t]
                v1 = paths[a1][t+1]
                for j in range(i + 1, num_agents):
                    a2 = agent_ids[j]
                    if t < len(paths[a2]) - 1:
                        u2 = paths[a2][t]
                        v2 = paths[a2][t+1]
                        if u1 == v2 and v1 == u2:
                            conflicts.append(('edge', a1, a2, u1, v1, t))
                            
    return conflicts

def find_first_conflict(paths):
    conflicts = find_all_conflicts(paths)
    return conflicts[0] if conflicts else None

class TCCBSTPlanner:
    def __init__(self, teams, agents_def, epsilon=0.1, grid_width=20, grid_height=24):
        # teams: list of dicts: {'id': int, 'name': str, 'agents': list of int, 'objective': 'min-sum' or 'min-max'}
        # agents_def: list of dicts: {'start': [x,y], 'goal': [x,y]}
        self.teams = teams
        self.agents_def = agents_def
        self.epsilon = epsilon
        self.grid_width = grid_width
        self.grid_height = grid_height
        
        self.low_level = SpaceTimeAStar(grid_width, grid_height)
        self.num_agents = len(agents_def)
        
    def compute_team_costs(self, paths):
        # Compute cost for each team based on its objective
        costs = []
        for team in self.teams:
            team_agents = team['agents']
            agent_costs = []
            for a in team_agents:
                # Path cost is path length - 1
                agent_costs.append(len(paths[a]) - 1)
            
            if team['objective'] == 'min-sum':
                costs.append(sum(agent_costs))
            elif team['objective'] == 'min-max':
                costs.append(max(agent_costs) if agent_costs else 0)
            else:
                costs.append(sum(agent_costs))
        return costs

    def compute_transformed_costs(self, team_costs, paths):
        # gf(Tj) = g(Tj) + epsilon * sum_{i not in Tj} g(pi^i)
        transformed_costs = []
        individual_costs = {a: len(paths[a]) - 1 for a in range(self.num_agents)}
        
        for j, team in enumerate(self.teams):
            team_agents = set(team['agents'])
            sum_outside = sum(individual_costs[a] for a in range(self.num_agents) if a not in team_agents)
            g_f = team_costs[j] + self.epsilon * sum_outside
            transformed_costs.append(g_f)
        return transformed_costs

    def count_inter_team_conflicts(self, paths):
        conflicts = find_all_conflicts(paths)
        count = 0
        agent_team = {}
        for team in self.teams:
            for a in team['agents']:
                agent_team[a] = team['id']
                
        for c in conflicts:
            a1, a2 = c[1], c[2]
            t1 = agent_team.get(a1)
            t2 = agent_team.get(a2)
            if t1 is not None and t2 is not None and t1 != t2:
                count += 1
        return count

    def plan(self, max_nodes=1000, time_limit=60, dynamic_obstacles=None):
        start_time = time.time()
        # Open list is a heap of nodes
        open_list = []
        # Solutions set C: contains list of tuples (untransformed_cost, CTNode)
        C = []
        
        # Build root node
        root = CTNode()
        for a in range(self.num_agents):
            start = tuple(self.agents_def[a]['start'])
            goal = tuple(self.agents_def[a]['goal'])
            path = self.low_level.find_path(start, goal, set(), set(), dynamic_obstacles)
            if path is None:
                return [] # Unsolvable
            root.paths[a] = path
            root.constraints[a] = set()
            
        root.cost_vector = self.compute_team_costs(root.paths)
        root.transformed_cost_vector = self.compute_transformed_costs(root.cost_vector, root.paths)
        root.num_conflicts = len(find_all_conflicts(root.paths))
        root.num_inter_team_conflicts = self.count_inter_team_conflicts(root.paths)
        
        heapq.heappush(open_list, root)
        
        nodes_expanded = 0
        
        while open_list and (time.time() - start_time < time_limit) and (nodes_expanded < max_nodes):
            curr_node = heapq.heappop(open_list)
            nodes_expanded += 1
            
            # Dominance check against solutions in C
            is_dominated = False
            for sol_cost, _ in C:
                if dominates(sol_cost, curr_node.cost_vector) or sol_cost == curr_node.cost_vector:
                    is_dominated = True
                    break
            if is_dominated:
                continue
                
            # Find conflicts
            conflict = find_first_conflict(curr_node.paths)
            
            if conflict is None:
                # Solution found!
                # Check if it is dominated by any solution in C
                is_sol_dominated = False
                C_new = []
                for sol_cost, sol_node in C:
                    if dominates(sol_cost, curr_node.cost_vector):
                        is_sol_dominated = True
                        C_new.append((sol_cost, sol_node))
                    elif dominates(curr_node.cost_vector, sol_cost):
                        # The new solution dominates the existing one, so discard the existing one
                        pass
                    else:
                        # Non-dominating
                        C_new.append((sol_cost, sol_node))
                        
                if not is_sol_dominated:
                    C_new.append((curr_node.cost_vector, curr_node))
                    C = C_new
                continue
                
            # Resolve conflict: split
            # conflict can be ('vertex', a1, a2, (x, y), t)
            # or ('edge', a1, a2, (x1, y1), (x2, y2), t)
            if conflict[0] == 'vertex':
                _, a1, a2, pos, t = conflict
                constraints_to_add = [
                    (a1, ('vertex', pos, t)),
                    (a2, ('vertex', pos, t))
                ]
            else: # edge conflict
                _, a1, a2, u, v, t = conflict
                constraints_to_add = [
                    (a1, ('edge', u, v, t)),
                    (a2, ('edge', v, u, t)) # For a2, moving from v to u at t is forbidden
                ]
                
            for agent_to_constrain, constr in constraints_to_add:
                child = curr_node.copy()
                child.constraints[agent_to_constrain].add(constr)
                
                # Re-plan for the constrained agent
                start = tuple(self.agents_def[agent_to_constrain]['start'])
                goal = tuple(self.agents_def[agent_to_constrain]['goal'])
                
                # Convert agent constraints to structure expected by A*
                vertex_constrs = set()
                edge_constrs = set()
                for c in child.constraints[agent_to_constrain]:
                    if c[0] == 'vertex':
                        vertex_constrs.add((c[1][0], c[1][1], c[2]))
                    elif c[0] == 'edge':
                        edge_constrs.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
                        
                new_path = self.low_level.find_path(start, goal, vertex_constrs, edge_constrs, dynamic_obstacles)
                
                if new_path is not None:
                    child.paths[agent_to_constrain] = new_path
                    child.cost_vector = self.compute_team_costs(child.paths)
                    child.transformed_cost_vector = self.compute_transformed_costs(child.cost_vector, child.paths)
                    child.num_conflicts = len(find_all_conflicts(child.paths))
                    child.num_inter_team_conflicts = self.count_inter_team_conflicts(child.paths)
                    
                    # Dominance check before pushing to open
                    child_dominated = False
                    for sol_cost, _ in C:
                        if dominates(sol_cost, child.cost_vector) or sol_cost == child.cost_vector:
                            child_dominated = True
                            break
                    if not child_dominated:
                        heapq.heappush(open_list, child)
                        
        # Return C containing non-dominated solutions
        # Return format: list of paths dicts
        solutions = []
        for _, node in C:
            solutions.append({
                'paths': node.paths,
                'cost_vector': node.cost_vector,
                'nodes_expanded': nodes_expanded,
                'runtime': time.time() - start_time
            })
        return solutions
