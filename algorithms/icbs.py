import heapq
import time
from algorithms.search_node import CTNode
from algorithms.a_star import SpaceTimeAStar
from algorithms.tc_cbs import find_all_conflicts, get_agent_pos_at_t

class ICBSPlanner:
    def __init__(self, agents_def, grid_width=20, grid_height=24):
        self.agents_def = agents_def
        self.grid_width = grid_width
        self.grid_height = grid_height
        
        self.low_level = SpaceTimeAStar(grid_width, grid_height)
        self.num_agents = len(agents_def)

    def compute_total_cost(self, paths):
        return sum(len(p) - 1 for p in paths.values())

    def classify_conflict(self, node, conflict, dynamic_obstacles=None):
        # conflict is ('vertex', a1, a2, pos, t) or ('edge', a1, a2, u, v, t)
        if conflict[0] == 'vertex':
            _, a1, a2, pos, t = conflict
            constr1 = ('vertex', pos, t)
            constr2 = ('vertex', pos, t)
        else:
            _, a1, a2, u, v, t = conflict
            constr1 = ('edge', u, v, t)
            constr2 = ('edge', v, u, t)
            
        # Re-plan for a1 with constr1
        v_constrs1 = set()
        e_constrs1 = set()
        for c in node.constraints.get(a1, set()):
            if c[0] == 'vertex': v_constrs1.add((c[1][0], c[1][1], c[2]))
            elif c[0] == 'edge': e_constrs1.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
        # Add the new constraint
        if constr1[0] == 'vertex': v_constrs1.add((constr1[1][0], constr1[1][1], constr1[2]))
        elif constr1[0] == 'edge': e_constrs1.add((constr1[1][0], constr1[1][1], constr1[2][0], constr1[2][1], constr1[3]))
        
        start1 = tuple(self.agents_def[a1]['start'])
        goal1 = tuple(self.agents_def[a1]['goal'])
        path1 = self.low_level.find_path(start1, goal1, v_constrs1, e_constrs1, dynamic_obstacles)
        cost1 = len(path1) - 1 if path1 else float('inf')
        
        # Re-plan for a2 with constr2
        v_constrs2 = set()
        e_constrs2 = set()
        for c in node.constraints.get(a2, set()):
            if c[0] == 'vertex': v_constrs2.add((c[1][0], c[1][1], c[2]))
            elif c[0] == 'edge': e_constrs2.add((c[1][0], c[1][1], c[2][0], c[2][1], c[3]))
        # Add the new constraint
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
        
        # To avoid doing too many A* runs, only classify up to the first 5 conflicts
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
            
        # Fallback if list is empty or classifications failed
        if conflicts:
            return 'unknown', conflicts[0], (None, None)
        return 'none', None, (None, None)

    def plan(self, max_nodes=1000, time_limit=60, dynamic_obstacles=None):
        start_time = time.time()
        open_list = []
        
        # Root node
        root = CTNode()
        for a in range(self.num_agents):
            start = tuple(self.agents_def[a]['start'])
            goal = tuple(self.agents_def[a]['goal'])
            path = self.low_level.find_path(start, goal, set(), set(), dynamic_obstacles)
            if path is None:
                return None # Unsolvable
            root.paths[a] = path
            root.constraints[a] = set()
            
        # For single agent MAPF, cost is a scalar (represented as a 1D vector here for sorting)
        total_cost = self.compute_total_cost(root.paths)
        root.cost_vector = [total_cost]
        root.transformed_cost_vector = [total_cost]
        
        heapq.heappush(open_list, root)
        nodes_expanded = 0
        
        while open_list and (time.time() - start_time < time_limit) and (nodes_expanded < max_nodes):
            curr_node = heapq.heappop(open_list)
            nodes_expanded += 1
            
            conflicts = find_all_conflicts(curr_node.paths)
            if not conflicts:
                # Found solution!
                return {
                    'paths': curr_node.paths,
                    'cost': self.compute_total_cost(curr_node.paths),
                    'nodes_expanded': nodes_expanded,
                    'runtime': time.time() - start_time
                }
                
            # Prioritize conflict
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
                
            # Try bypassing for non-cardinal conflicts
            bypass_successful = False
            if cardinality == 'non-cardinal':
                # Bypassing rule: if a replanned path has the same cost and resolves the conflict, 
                # we can update in-place without branching.
                for idx, (agent_id, constr, replanned_path) in enumerate(constraints_to_add):
                    if replanned_path is not None:
                        orig_cost = len(curr_node.paths[agent_id]) - 1
                        new_cost = len(replanned_path) - 1
                        
                        if orig_cost == new_cost:
                            # Verify if the new path has fewer conflicts
                            temp_paths = dict(curr_node.paths)
                            temp_paths[agent_id] = replanned_path
                            new_conflicts = find_all_conflicts(temp_paths)
                            
                            # If new path has fewer conflicts or resolves this specific conflict
                            if len(new_conflicts) < len(conflicts):
                                curr_node.paths[agent_id] = replanned_path
                                # Reset cost vector
                                total_cost = self.compute_total_cost(curr_node.paths)
                                curr_node.cost_vector = [total_cost]
                                curr_node.transformed_cost_vector = [total_cost]
                                
                                # Add the constraint to the node just in case
                                if agent_id not in curr_node.constraints:
                                    curr_node.constraints[agent_id] = set()
                                curr_node.constraints[agent_id].add(constr)
                                
                                heapq.heappush(open_list, curr_node)
                                bypass_successful = True
                                break
                                
            if bypass_successful:
                continue
                
            # If bypass was not successful or conflict was cardinal/semi-cardinal, branch
            for agent_id, constr, replanned_path in constraints_to_add:
                child = curr_node.copy()
                child.constraints[agent_id].add(constr)
                
                # If we already have the replanned path from classification, use it!
                if replanned_path is not None:
                    child.paths[agent_id] = replanned_path
                else:
                    # Otherwise, re-plan
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
                    # Update cost
                    child_cost = self.compute_total_cost(child.paths)
                    child.cost_vector = [child_cost]
                    child.transformed_cost_vector = [child_cost]
                    heapq.heappush(open_list, child)
                    
        return None
