from typing import Dict, List, Tuple, Set, Optional

class CTNode:
    def __init__(
        self,
        paths: Optional[Dict[int, List[Tuple[int, int]]]] = None,
        constraints: Optional[Dict[int, Set[Tuple]]] = None,
        cost_vector: Optional[List[int]] = None,
        transformed_cost_vector: Optional[List[float]] = None,
        num_conflicts: int = 0,
        num_inter_team_conflicts: int = 0,
        depth: int = 0
    ) -> None:
        """Initialize a Constraint Tree Node.

        Args:
            paths: Dict mapping agent_id to list of (x, y) coordinates representing its path.
            constraints: Dict mapping agent_id to a set of constraints.
            cost_vector: Individual/team cost components.
            transformed_cost_vector: Transformed objective cost vector.
            num_conflicts: Total number of conflicts in the current paths.
            num_inter_team_conflicts: Total number of inter-team conflicts.
            depth: Depth of the node in the high-level constraint tree.
        """
        # paths: dict mapping agent_id -> list of (x,y)
        self.paths: Dict[int, List[Tuple[int, int]]] = paths if paths is not None else {}
        # constraints: dict mapping agent_id -> set of constraints
        # Each constraint can be:
        # - ('vertex', (x, y), t)
        # - ('edge', (x1, y1), (x2, y2), t)
        self.constraints: Dict[int, Set[Tuple]] = constraints if constraints is not None else {}
        
        self.cost_vector: List[int] = cost_vector if cost_vector is not None else []
        self.transformed_cost_vector: List[float] = transformed_cost_vector if transformed_cost_vector is not None else []
        self.num_conflicts: int = num_conflicts
        self.num_inter_team_conflicts: int = num_inter_team_conflicts
        self.depth: int = depth
        
    def __lt__(self, other: 'CTNode') -> bool:
        # Lexicographical comparison of transformed cost vectors
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
        
    def copy(self) -> 'CTNode':
        new_paths = {k: list(v) for k, v in self.paths.items()}
        new_constraints = {k: set(v) for k, v in self.constraints.items()}
        return CTNode(
            paths=new_paths,
            constraints=new_constraints,
            cost_vector=list(self.cost_vector),
            transformed_cost_vector=list(self.transformed_cost_vector),
            num_conflicts=self.num_conflicts,
            num_inter_team_conflicts=self.num_inter_team_conflicts,
            depth=self.depth
        )
