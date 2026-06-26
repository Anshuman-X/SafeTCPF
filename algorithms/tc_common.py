from typing import Dict, List, Tuple, Optional

def dominates(a: List[float], b: List[float]) -> bool:
    """Check if cost vector a dominates cost vector b.

    a dominates b if a is component-wise <= b and strictly < in at least one component.
    """
    all_le = all(x <= y + 1e-9 for x, y in zip(a, b))
    any_lt = any(x < y - 1e-9 for x, y in zip(a, b))
    return all_le and any_lt

def get_agent_pos_at_t(path: List[Tuple[int, int]], t: int) -> Tuple[int, int]:
    """Get the position of an agent at time step t.

    If t exceeds the path length, the agent is assumed to remain at its goal position.
    """
    if t < len(path):
        return path[t]
    return path[-1]

def find_all_conflicts(paths: Dict[int, List[Tuple[int, int]]]) -> List[Tuple]:
    """Find all vertex and edge conflicts among a set of paths.

    Returns a list of conflicts where each conflict is represented as:
    - ('vertex', a1, a2, (x, y), t)
    - ('edge', a1, a2, (x1, y1), (x2, y2), t)
    """
    conflicts = []
    num_agents = len(paths)
    agent_ids = list(paths.keys())
    
    max_path_len = max(len(p) for p in paths.values()) if paths else 0
    
    for t in range(max_path_len + 5):  # Check slightly beyond to ensure goal occupancy safety
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

def find_first_conflict(paths: Dict[int, List[Tuple[int, int]]]) -> Optional[Tuple]:
    """Find the first conflict in the given paths."""
    conflicts = find_all_conflicts(paths)
    return conflicts[0] if conflicts else None

def compute_team_costs(teams: List[Dict], paths: Dict[int, List[Tuple[int, int]]]) -> List[int]:
    """Compute cost for each team based on its objective (min-sum or min-max)."""
    costs = []
    for team in teams:
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

def compute_transformed_costs(
    teams: List[Dict],
    team_costs: List[int],
    paths: Dict[int, List[Tuple[int, int]]],
    epsilon: float,
    num_agents: int
) -> List[float]:
    """Compute transformed team costs.

    gf(Tj) = g(Tj) + epsilon * sum_{i not in Tj} g(pi^i)
    """
    transformed_costs = []
    individual_costs = {a: len(paths[a]) - 1 for a in range(num_agents)}
    
    for j, team in enumerate(teams):
        team_agents = set(team['agents'])
        sum_outside = sum(individual_costs[a] for a in range(num_agents) if a not in team_agents)
        g_f = team_costs[j] + epsilon * sum_outside
        transformed_costs.append(g_f)
    return transformed_costs

def count_inter_team_conflicts(teams: List[Dict], paths: Dict[int, List[Tuple[int, int]]]) -> int:
    """Count conflicts that involve agents from different teams."""
    conflicts = find_all_conflicts(paths)
    count = 0
    agent_team = {}
    for team in teams:
        for a in team['agents']:
            agent_team[a] = team['id']
            
    for c in conflicts:
        a1, a2 = c[1], c[2]
        t1 = agent_team.get(a1)
        t2 = agent_team.get(a2)
        if t1 is not None and t2 is not None and t1 != t2:
            count += 1
    return count
