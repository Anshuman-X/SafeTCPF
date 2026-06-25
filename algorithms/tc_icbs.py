import heapq
import time
from algorithms.search_node import CTNode
from algorithms.a_star import SpaceTimeAStar
from algorithms.tc_cbs import find_all_conflicts, dominates

class TCICBSPlanner:
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
        costs = []
        for team in self.teams:
            team_agents = team['agents']
            agent_costs = []
            for a in team_agents:
                agent_costs.append(len(paths[a]) - 1)
            
            if team['objective'] == 'min-sum':
                costs.append(sum(agent_costs))
            elif team['objective'] == 'min-max':
                costs.append(max(agent_costs) if agent_costs else 0)
            else:
                costs.append(sum(agent_costs))
        return costs

    def compute_transformed_costs(self, team_costs, paths):
        transformed_costs = []
        individual_costs = {a: len(paths[a]) - 1 for a in range(self.num_agents)}
        
        for j, team in enumerate(self.teams):
            team_agents = set(team['agents'])
            sum_outside = sum(individual_costs[a] for a in range(self.num_agents) if a not in team_agents)
            g_f = team_costs[j] + self.epsilon * sum_outside
            transformed_costs.append(g_f)
        return transformed_costs

    def classify_conflict(self, node, conflict, dynamic_obstacles=None):
        if conflict[0] == 'vertex':
            _, a1, a2, pos, t = conflict
            constr1 = ('vertex', pos, t)
            constr2 = ('vertex', pos, t)
        else:
            _, a1, a2, u, v, t = conflict
            constr1 = ('edge', u, v, t)
            constr2 = ('edge', v, u, t)
            
        # Re-plan for a1
        v_constrs1 = set()
        e_constrs1 = set()
        for c in node.constraints.get(a1, set()):
            if c[0] == 'vertex': v_constrs1.add((c[1][0], c[1][1], c[2]))
            elif c[0] == 'edge': e_constrs1.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
        if constr1[0] == 'vertex': v_constrs1.add((constr1[1][0], constr1[1][1], constr1[2]))
        elif constr1[0] == 'edge': e_constrs1.add((constr1[1][0], constr1[1][1], constr1[2][0], constr1[2][1], constr1[3]))
        
        start1 = tuple(self.agents_def[a1]['start'])
        goal1 = tuple(self.agents_def[a1]['goal'])
        path1 = self.low_level.find_path(start1, goal1, v_constrs1, e_constrs1, dynamic_obstacles)
        cost1 = len(path1) - 1 if path1 else float('inf')
        
        # Re-plan for a2
        v_constrs2 = set()
        e_constrs2 = set()
        for c in node.constraints.get(a2, set()):
            if c[0] == 'vertex': v_constrs2.add((c[1][0], c[1][1], c[2]))
            elif c[0] == 'edge': e_constrs2.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
        if constr2[0] == 'vertex': v_constrs2.add((constr2[1][0], constr2[1][1], constr2[2]))
        elif constr2[0] == 'edge': e_constrs2.add((constr2[1][0], constr2[1][1], constr2[2][0], constr2[2][1], constr2[3]))
        
        start2 = tuple(self.agents_def[a2]['start'])
        goal2 = tuple(self.agents_def[a2]['goal'])
        path2 = self.low_level.find_path(start2, goal2, v_constrs2, e_constrs2, dynamic_obstacles)
        cost2 = len(path2) - 1 if path2 else float('inf')
        
        orig_cost1 = len(node.paths[a1]) - 1
        orig_cost2 = len(node.paths[a2]) - 1
        
        is_card1 = cost1 > orig_cost1
        is_card2 = cost2 > orig_cost2
        
        if is_card1 and is_card2:
            return 'cardinal', (path1, path2)
        elif is_card1 or is_card2:
            return 'semi-cardinal', (path1, path2)
        else:
            return 'non-cardinal', (path1, path2)

    def select_best_conflict(self, node, conflicts, dynamic_obstacles=None):
        cardinal_conflicts = []
        semi_conflicts = []
        non_conflicts = []
        
        for conflict in conflicts[:5]:
            cardinality, replanned_paths = self.classify_conflict(node, conflict, dynamic_obstacles)
            if cardinality == 'cardinal':
                cardinal_conflicts.append((conflict, replanned_paths))
            elif cardinality == 'semi-cardinal':
                semi_conflicts.append((conflict, replanned_paths))
            else:
                non_conflicts.append((conflict, replanned_paths))
                
        if cardinal_conflicts:
            return 'cardinal', cardinal_conflicts[0][0], cardinal_conflicts[0][1]
        elif semi_conflicts:
            return 'semi-cardinal', semi_conflicts[0][0], semi_conflicts[0][1]
        elif non_conflicts:
            return 'non-cardinal', non_conflicts[0][0], non_conflicts[0][1]
            
        if conflicts:
            return 'unknown', conflicts[0], (None, None)
        return 'none', None, (None, None)

    def plan(self, max_nodes=1000, time_limit=60, dynamic_obstacles=None):
        start_time = time.time()
        open_list = []
        C = [] # Non-dominated solutions: list of tuples (untransformed_cost, CTNode)
        
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
        
        heapq.heappush(open_list, root)
        nodes_expanded = 0
        
        while open_list and (time.time() - start_time < time_limit) and (nodes_expanded < max_nodes):
            curr_node = heapq.heappop(open_list)
            nodes_expanded += 1
            
            # Dominance check
            is_dominated = False
            for sol_cost, _ in C:
                if dominates(sol_cost, curr_node.cost_vector) or sol_cost == curr_node.cost_vector:
                    is_dominated = True
                    break
            if is_dominated:
                continue
                
            conflicts = find_all_conflicts(curr_node.paths)
            if not conflicts:
                # Found a Pareto-optimal solution!
                is_sol_dominated = False
                C_new = []
                for sol_cost, sol_node in C:
                    if dominates(sol_cost, curr_node.cost_vector):
                        is_sol_dominated = True
                        C_new.append((sol_cost, sol_node))
                    elif dominates(curr_node.cost_vector, sol_cost):
                        # New dominates existing
                        pass
                    else:
                        C_new.append((sol_cost, sol_node))
                        
                if not is_sol_dominated:
                    C_new.append((curr_node.cost_vector, curr_node))
                    C = C_new
                continue
                
            # Classify and select best conflict
            cardinality, conflict, (path1, path2) = self.select_best_conflict(curr_node, conflicts, dynamic_obstacles)
            
            # Setup split constraints
            if conflict[0] == 'vertex':
                _, a1, a2, pos, t = conflict
                constraints_to_add = [
                    (a1, ('vertex', pos, t), path1),
                    (a2, ('vertex', pos, t), path2)
                ]
            else:
                _, a1, a2, u, v, t = conflict
                constraints_to_add = [
                    (a1, ('edge', u, v, t), path1),
                    (a2, ('edge', v, u, t), path2)
                ]
                
            # Bypassing for non-cardinal conflicts
            bypass_successful = False
            if cardinality == 'non-cardinal':
                for agent_id, constr, replanned_path in constraints_to_add:
                    if replanned_path is not None:
                        orig_cost = len(curr_node.paths[agent_id]) - 1
                        new_cost = len(replanned_path) - 1
                        
                        if orig_cost == new_cost:
                            temp_paths = dict(curr_node.paths)
                            temp_paths[agent_id] = replanned_path
                            new_conflicts = find_all_conflicts(temp_paths)
                            
                            if len(new_conflicts) < len(conflicts):
                                curr_node.paths[agent_id] = replanned_path
                                curr_node.cost_vector = self.compute_team_costs(curr_node.paths)
                                curr_node.transformed_cost_vector = self.compute_transformed_costs(curr_node.cost_vector, curr_node.paths)
                                
                                if agent_id not in curr_node.constraints:
                                    curr_node.constraints[agent_id] = set()
                                curr_node.constraints[agent_id].add(constr)
                                
                                heapq.heappush(open_list, curr_node)
                                bypass_successful = True
                                break
                                
            if bypass_successful:
                continue
                
            # Branching
            for agent_id, constr, replanned_path in constraints_to_add:
                child = curr_node.copy()
                child.constraints[agent_id].add(constr)
                
                if replanned_path is not None:
                    child.paths[agent_id] = replanned_path
                else:
                    start = tuple(self.agents_def[agent_id]['start'])
                    goal = tuple(self.agents_def[agent_id]['goal'])
                    
                    vertex_constrs = set()
                    edge_constrs = set()
                    for c in child.constraints[agent_id]:
                        if c[0] == 'vertex': vertex_constrs.add((c[1][0], c[1][1], c[2]))
                        elif c[0] == 'edge': edge_constrs.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
                        
                    new_path = self.low_level.find_path(start, goal, vertex_constrs, edge_constrs, dynamic_obstacles)
                    if new_path is not None:
                        child.paths[agent_id] = new_path
                    else:
                        child = None
                        
                if child is not None:
                    child.cost_vector = self.compute_team_costs(child.paths)
                    child.transformed_cost_vector = self.compute_transformed_costs(child.cost_vector, child.paths)
                    
                    # Dominance check
                    child_dominated = False
                    for sol_cost, _ in C:
                        if dominates(sol_cost, child.cost_vector) or sol_cost == child.cost_vector:
                            child_dominated = True
                            break
                    if not child_dominated:
                        heapq.heappush(open_list, child)
                        
        solutions = []
        for _, node in C:
            solutions.append({
                'paths': node.paths,
                'cost_vector': node.cost_vector,
                'nodes_expanded': nodes_expanded,
                'runtime': time.time() - start_time
            })
        return solutions
