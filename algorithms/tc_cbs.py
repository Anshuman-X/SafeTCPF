import heapq
import time
from algorithms.search_node import CTNode
from algorithms.a_star import SpaceTimeAStar
from algorithms.tc_common import (
    dominates,
    find_all_conflicts,
    find_first_conflict,
    compute_team_costs,
    compute_transformed_costs,
    count_inter_team_conflicts,
)

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
        """Delegate to shared tc_common implementation."""
        return compute_team_costs(self.teams, paths)

    def compute_transformed_costs(self, team_costs, paths):
        """Delegate to shared tc_common implementation."""
        return compute_transformed_costs(self.teams, team_costs, paths, self.epsilon, self.num_agents)

    def count_inter_team_conflicts(self, paths):
        """Delegate to shared tc_common implementation."""
        return count_inter_team_conflicts(self.teams, paths)

    def plan(self, max_nodes=1000, time_limit=60, dynamic_obstacles=None):
        start_time = time.time()
        # Open list is a heap of nodes
        open_list = []
        # Solutions set C: contains list of tuples (untransformed_cost, CTNode)
        C = []
        
        # Track search metrics (Objective 12)
        generated_nodes = 1
        expanded_nodes = 0
        max_open_list_size = 0
        max_depth = 0
        num_replans = 0

        # Build root node
        root = CTNode()
        for a in range(self.num_agents):
            start = tuple(self.agents_def[a]['start'])
            goal = tuple(self.agents_def[a]['goal'])
            num_replans += 1
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
        max_open_list_size = max(max_open_list_size, len(open_list))
        
        nodes_expanded = 0
        
        while open_list and (time.time() - start_time < time_limit) and (nodes_expanded < max_nodes):
            curr_node = heapq.heappop(open_list)
            nodes_expanded += 1
            expanded_nodes += 1
            max_depth = max(max_depth, curr_node.depth)
            
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
                child.depth = curr_node.depth + 1
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
                        
                num_replans += 1
                new_path = self.low_level.find_path(start, goal, vertex_constrs, edge_constrs, dynamic_obstacles)
                
                if new_path is not None:
                    generated_nodes += 1
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
                        max_open_list_size = max(max_open_list_size, len(open_list))
                        
        # Return C containing non-dominated solutions
        solutions = []
        for _, node in C:
            solutions.append({
                'paths': node.paths,
                'cost_vector': node.cost_vector,
                'nodes_expanded': nodes_expanded,
                'generated_nodes': generated_nodes,
                'expanded_nodes': expanded_nodes,
                'max_depth': max_depth,
                'max_open_list_size': max_open_list_size,
                'num_replans': num_replans,
                'runtime': time.time() - start_time
            })
        return solutions
