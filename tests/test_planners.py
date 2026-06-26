import unittest
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algorithms.a_star import SpaceTimeAStar

class TestSpaceTimeAStar(unittest.TestCase):
    def setUp(self):
        self.planner = SpaceTimeAStar()

    def test_simple_path(self):
        # East to West path on westbound lane
        start = (19, 12)
        goal = (0, 12)
        path = self.planner.find_path(start, goal, set(), set(), max_t=50)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_vertex_constraint(self):
        # Start at (19, 12), goal at (15, 12)
        # Force a wait or detour by blocking (17, 12) at t=2
        start = (19, 12)
        goal = (15, 12)
        
        # Standard path would be: (19,12)[t=0] -> (18,12)[t=1] -> (17,12)[t=2] -> (16,12)[t=3] -> (15,12)[t=4]
        vertex_constraints = {(17, 12, 2)}
        path = self.planner.find_path(start, goal, vertex_constraints, set(), max_t=50)
        
        self.assertIsNotNone(path)
        # Check that it did not visit (17, 12) at t=2
        self.assertNotEqual(path[2], (17, 12))

from algorithms.tc_cbs import TCCBSTPlanner

class TestTCCBSTPlanner(unittest.TestCase):
    def test_two_agent_intersection_conflict(self):
        # Setup teams
        teams = [
            {'id': 1, 'name': 'Team1', 'agents': [0], 'objective': 'min-sum'},
            {'id': 2, 'name': 'Team2', 'agents': [1], 'objective': 'min-sum'}
        ]
        # Setup agents
        agents_def = [
            {'start': [15, 12], 'goal': [5, 12]},  # Westbound (horizontal)
            {'start': [10, 7], 'goal': [10, 17]}   # Northbound (vertical)
        ]
        
        planner = TCCBSTPlanner(teams, agents_def, epsilon=0.1)
        solutions = planner.plan(max_nodes=100, time_limit=10)
        
        self.assertIsNotNone(solutions)
        self.assertTrue(len(solutions) > 0)
        
        # Verify that all solutions are conflict-free
        from algorithms.tc_common import find_first_conflict
        for sol in solutions:
            paths = sol['paths']
            conflict = find_first_conflict(paths)
            self.assertIsNone(conflict)

from algorithms.icbs import ICBSPlanner

class TestICBSPlanner(unittest.TestCase):
    def test_icbs_intersection_conflict(self):
        # Setup agents
        agents_def = [
            {'start': [15, 12], 'goal': [5, 12]},  # Westbound (horizontal)
            {'start': [10, 7], 'goal': [10, 17]}   # Northbound (vertical)
        ]
        
        planner = ICBSPlanner(agents_def)
        solution = planner.plan(max_nodes=100, time_limit=10)
        
        self.assertIsNotNone(solution)
        self.assertTrue(solution['cost'] > 0)
        
        # Verify conflict-free
        from algorithms.tc_common import find_first_conflict
        conflict = find_first_conflict(solution['paths'])
        self.assertIsNone(conflict)

from algorithms.tc_icbs import TCICBSPlanner

class TestTCICBSPlanner(unittest.TestCase):
    def test_tc_icbs_intersection_conflict(self):
        # Setup teams
        teams = [
            {'id': 1, 'name': 'Team1', 'agents': [0], 'objective': 'min-sum'},
            {'id': 2, 'name': 'Team2', 'agents': [1], 'objective': 'min-sum'}
        ]
        # Setup agents
        agents_def = [
            {'start': [15, 12], 'goal': [5, 12]},  # Westbound
            {'start': [10, 7], 'goal': [10, 17]}   # Northbound
        ]
        
        planner = TCICBSPlanner(teams, agents_def, epsilon=0.1)
        solutions = planner.plan(max_nodes=100, time_limit=10)
        
        self.assertIsNotNone(solutions)
        self.assertTrue(len(solutions) > 0)
        
        # Verify conflict-free
        from algorithms.tc_common import find_first_conflict
        for sol in solutions:
            conflict = find_first_conflict(sol['paths'])
            self.assertIsNone(conflict)

if __name__ == "__main__":
    unittest.main()



