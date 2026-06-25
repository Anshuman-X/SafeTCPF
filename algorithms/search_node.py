class CTNode:
    def __init__(self, paths=None, constraints=None, cost_vector=None, transformed_cost_vector=None, num_conflicts=0, num_inter_team_conflicts=0):
        # paths: dict mapping agent_id -> list of (x,y)
        self.paths = paths if paths is not None else {}
        # constraints: dict mapping agent_id -> set of constraints
        # Each constraint can be:
        # - ('vertex', (x, y), t)
        # - ('edge', (x1, y1), (x2, y2), t)
        self.constraints = constraints if constraints is not None else {}
        
        self.cost_vector = cost_vector if cost_vector is not None else []
        self.transformed_cost_vector = transformed_cost_vector if transformed_cost_vector is not None else []
        self.num_conflicts = num_conflicts
        self.num_inter_team_conflicts = num_inter_team_conflicts
        
    def __lt__(self, other):
        # Lexicographical comparison of transformed cost vectors
        # If they are of different lengths (which shouldn't happen), compare normally
        for a, b in zip(self.transformed_cost_vector, other.transformed_cost_vector):
            if abs(a - b) > 1e-9:
                return a < b
        # Secondary tie-breaker: fewer inter-team conflicts is better (Team-Aware Heuristic)
        if self.num_inter_team_conflicts != other.num_inter_team_conflicts:
            return self.num_inter_team_conflicts < other.num_inter_team_conflicts
        # Tertiary tie-breaker: fewer total conflicts is better
        if self.num_conflicts != other.num_conflicts:
            return self.num_conflicts < other.num_conflicts
        return len(self.transformed_cost_vector) < len(other.transformed_cost_vector)
        
    def copy(self):
        new_paths = {k: list(v) for k, v in self.paths.items()}
        new_constraints = {k: set(v) for k, v in self.constraints.items()}
        return CTNode(
            paths=new_paths,
            constraints=new_constraints,
            cost_vector=list(self.cost_vector),
            transformed_cost_vector=list(self.transformed_cost_vector),
            num_conflicts=self.num_conflicts,
            num_inter_team_conflicts=self.num_inter_team_conflicts
        )

