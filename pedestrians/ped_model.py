class PedestrianModel:
    def __init__(self, grid_width=20, grid_height=24):
        self.grid_width = grid_width
        self.grid_height = grid_height
        
    def generate_pedestrians(self, density="medium", sim_steps=50):
        # Generates pedestrian paths based on density
        # Returns a list of dicts: {'id': str, 'path': list of (x,y), 'start_time': int}
        pedestrians = []
        
        # Define crosswalks
        # 1. North Crosswalk: y=14, x from 7 to 12
        north_eastbound = [(x, 14) for x in range(7, 13)]
        north_westbound = [(x, 14) for x in range(12, 6, -1)]
        
        # 2. South Crosswalk: y=9, x from 7 to 12
        south_eastbound = [(x, 9) for x in range(7, 13)]
        south_westbound = [(x, 9) for x in range(12, 6, -1)]
        
        # 3. East Crosswalk: x=12, y from 9 to 14
        east_northbound = [(12, y) for y in range(9, 15)]
        east_southbound = [(12, y) for y in range(14, 8, -1)]
        
        # 4. West Crosswalk: x=7, y from 9 to 14
        west_northbound = [(7, y) for y in range(9, 15)]
        west_southbound = [(7, y) for y in range(14, 8, -1)]
        
        crosswalks = [
            north_eastbound, north_westbound,
            south_eastbound, south_westbound,
            east_northbound, east_southbound,
            west_northbound, west_southbound
        ]
        
        # Determine number of pedestrians and spacing based on density
        if density == "low":
            # 2 pedestrians
            spawn_times = [2, 6]
            paths_to_use = [0, 2] # north_eastbound, south_eastbound
        elif density == "medium":
            # 6 pedestrians
            spawn_times = [1, 3, 5, 7, 9, 11]
            paths_to_use = [0, 1, 2, 3, 4, 6]
        elif density == "high":
            # 12 pedestrians
            spawn_times = [0, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17]
            paths_to_use = [0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3]
        else:
            spawn_times = []
            paths_to_use = []
            
        for idx, (t_start, path_idx) in enumerate(zip(spawn_times, paths_to_use)):
            base_path = crosswalks[path_idx]
            ped_id = f"ped_{idx}"
            
            # Construct a step-by-step path over the simulation
            # Pedestrian stays at start until spawn time, then walks, then stays at goal
            ped_path = []
            for t in range(sim_steps):
                if t < t_start:
                    ped_path.append(base_path[0])
                elif t - t_start < len(base_path):
                    ped_path.append(base_path[t - t_start])
                else:
                    ped_path.append(base_path[-1])
                    
            pedestrians.append({
                'id': ped_id,
                'path': ped_path,
                'start_time': t_start
            })
            
        return pedestrians

    def get_occupancy_set(self, pedestrians):
        # Converts pedestrian paths into a set of (x, y, t) coordinates
        occupancy = set()
        for ped in pedestrians:
            for t, pos in enumerate(ped['path']):
                # To be safe, we block the pedestrian cell at time t, t-1, and t+1
                # This creates a safety buffer around pedestrians.
                occupancy.add((pos[0], pos[1], t))
                occupancy.add((pos[0], pos[1], max(0, t - 1)))
                occupancy.add((pos[0], pos[1], t + 1))
        return occupancy
