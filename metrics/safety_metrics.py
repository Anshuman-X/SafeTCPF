import numpy as np
import pandas as pd
import os

def get_agent_pos_at_t(path, t):
    if t < len(path):
        return path[t]
    return path[-1]

def calculate_metrics(vehicle_paths, pedestrian_paths=None, grid_width=20, grid_height=24):
    # vehicle_paths: dict of agent_id -> list of (x,y)
    # pedestrian_paths: list of dicts: {'id': str, 'path': list of (x,y)}
    
    if pedestrian_paths is None:
        pedestrian_paths = []
        
    num_vehicles = len(vehicle_paths)
    if num_vehicles == 0:
        return {}
        
    # 1. Travel Time (TT)
    travel_times = {}
    for a, path in vehicle_paths.items():
        travel_times[a] = len(path) - 1
        
    avg_travel_time = np.mean(list(travel_times.values()))
    
    # 2. Waiting Time (WT)
    waiting_times = {}
    for a, path in vehicle_paths.items():
        wt = 0
        for t in range(1, len(path)):
            if path[t] == path[t-1]:
                wt += 1
        waiting_times[a] = wt
        
    avg_waiting_time = np.mean(list(waiting_times.values()))
    
    # 3. Average Delay (AD)
    # Delay = Travel Time - Free Flow Travel Time (Manhattan distance from start to goal)
    delays = {}
    # We will assume start and goal are the first and last elements of the path
    for a, path in vehicle_paths.items():
        start = path[0]
        goal = path[-1]
        free_flow = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        delays[a] = max(0, (len(path) - 1) - free_flow)
        
    avg_delay = np.mean(list(delays.values()))
    
    # 4. Average Speed (AS)
    # Speed = (Travel Time - Waiting Time) / Travel Time
    speeds = {}
    for a, path in vehicle_paths.items():
        tt = travel_times[a]
        if tt > 0:
            speeds[a] = (tt - waiting_times[a]) / tt
        else:
            speeds[a] = 0.0
            
    avg_speed = np.mean(list(speeds.values()))
    
    # 5. Collision Count and Conflict Count (Near-misses)
    # Near-miss is when distance is <= 1.5 cells (meaning adjacent or diagonal adjacent)
    collision_count = 0
    conflict_count = 0
    
    max_t = max(len(p) for p in vehicle_paths.values())
    
    # Check vehicle-vehicle near-misses
    for t in range(max_t):
        positions = {}
        for a, path in vehicle_paths.items():
            positions[a] = get_agent_pos_at_t(path, t)
            
        agent_ids = list(vehicle_paths.keys())
        for i in range(len(agent_ids)):
            a1 = agent_ids[i]
            p1 = positions[a1]
            for j in range(i + 1, len(agent_ids)):
                a2 = agent_ids[j]
                p2 = positions[a2]
                
                # Exclude if both have reached their goals and stopped
                if t >= len(vehicle_paths[a1]) - 1 and t >= len(vehicle_paths[a2]) - 1:
                    continue
                    
                dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                if dist < 0.1:
                    collision_count += 1
                elif dist <= 1.5:
                    conflict_count += 1
                    
    # 6. Time-to-Collision (TTC)
    # Calculate minimum TTC at each step
    ttc_list = []
    for t in range(1, max_t):
        for a1 in vehicle_paths.keys():
            path1 = vehicle_paths[a1]
            p1 = np.array(get_agent_pos_at_t(path1, t))
            p1_prev = np.array(get_agent_pos_at_t(path1, t-1))
            v1 = p1 - p1_prev
            
            for a2 in vehicle_paths.keys():
                if a1 >= a2:
                    continue
                path2 = vehicle_paths[a2]
                p2 = np.array(get_agent_pos_at_t(path2, t))
                p2_prev = np.array(get_agent_pos_at_t(path2, t-1))
                v2 = p2 - p2_prev
                
                # Exclude if both at goal
                if t >= len(path1) - 1 and t >= len(path2) - 1:
                    continue
                    
                rel_pos = p2 - p1
                rel_vel = v1 - v2
                
                # If they are moving towards each other
                dot_prod = np.dot(rel_pos, rel_vel)
                if dot_prod > 0.0: # positive dot product means moving closer
                    rel_vel_norm_sq = np.dot(rel_vel, rel_vel)
                    if rel_vel_norm_sq > 1e-6:
                        # Project position onto velocity vector to find time of closest approach
                        ttc = dot_prod / rel_vel_norm_sq
                        ttc_list.append(ttc)
                        
    min_ttc = np.min(ttc_list) if ttc_list else 10.0 # Default safe value if no interactions
    avg_ttc = np.mean(ttc_list) if ttc_list else 10.0
    
    # 7. Post-Encroachment Time (PET)
    # Track the occupancy time list for each grid cell
    cell_occupancy = {} # (x, y) -> list of (time, agent_id)
    for a, path in vehicle_paths.items():
        for t, pos in enumerate(path):
            # Only record if the agent hasn't reached its goal (so it's still traversing)
            if t < len(path):
                if pos not in cell_occupancy:
                    cell_occupancy[pos] = []
                cell_occupancy[pos].append((t, a))
                
    pet_list = []
    for pos, occupancies in cell_occupancy.items():
        if len(occupancies) > 1:
            # Sort by time
            occupancies.sort()
            for k in range(len(occupancies) - 1):
                t1, a1 = occupancies[k]
                t2, a2 = occupancies[k+1]
                if a1 != a2: # Encroachment by different agents
                    pet = t2 - t1
                    pet_list.append(pet)
                    
    min_pet = np.min(pet_list) if pet_list else 10.0
    avg_pet = np.mean(pet_list) if pet_list else 10.0
    
    return {
        'avg_travel_time': float(avg_travel_time),
        'avg_waiting_time': float(avg_waiting_time),
        'avg_delay': float(avg_delay),
        'avg_speed': float(avg_speed),
        'collision_count': int(collision_count),
        'conflict_count': int(conflict_count),
        'min_ttc': float(min_ttc),
        'avg_ttc': float(avg_ttc),
        'min_pet': float(min_pet),
        'avg_pet': float(avg_pet)
    }

def save_metrics_to_csv(metrics_dict, file_path):
    # Ensure directory exists
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    df = pd.DataFrame([metrics_dict])
    df.to_csv(file_path, index=False)
