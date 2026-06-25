import numpy as np
import pandas as pd
import os

def get_agent_pos_at_t(path, t):
    if t < len(path):
        return path[t]
    return path[-1]

def calculate_pair_ttc(p1, v1, p2, v2, d=1.5):
    """Calculate mathematically correct Time-to-Collision (TTC) using quadratic formula.
    Assumes vehicles are represented as circles of radius R (where d = R1 + R2).
    """
    rel_pos = p2 - p1
    rel_vel = v2 - v1  # relative velocity w = v2 - v1
    
    dist = np.linalg.norm(rel_pos)
    if dist < d:
        return 0.0  # Already colliding or within safety distance
        
    # Solve: ||rel_pos + rel_vel * t||^2 = d^2
    # which is: a * t^2 + b * t + c = 0
    # where:
    # a = ||rel_vel||^2
    # b = 2 * (rel_pos . rel_vel)
    # c = ||rel_pos||^2 - d^2
    a = np.dot(rel_vel, rel_vel)
    b = 2.0 * np.dot(rel_pos, rel_vel)
    c = np.dot(rel_pos, rel_pos) - d * d
    
    if a < 1e-6:
        return float('inf')  # No relative motion
        
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        return float('inf')  # No collision path
        
    # For positive roots, we need b < 0 (moving closer)
    if b >= 0:
        return float('inf')
        
    t1 = (-b - np.sqrt(discriminant)) / (2.0 * a)
    t2 = (-b + np.sqrt(discriminant)) / (2.0 * a)
    
    roots = [r for r in [t1, t2] if r >= 0]
    if not roots:
        return float('inf')
    return min(roots)

def calculate_metrics(vehicle_paths, pedestrian_paths=None, grid_width=20, grid_height=24):
    """Computes fine-grained safety and efficiency metrics for the simulation paths."""
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
    delays = {}
    for a, path in vehicle_paths.items():
        start = path[0]
        goal = path[-1]
        free_flow = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
        delays[a] = max(0, (len(path) - 1) - free_flow)
        
    avg_delay = np.mean(list(delays.values()))
    
    # 4. Average Speed (AS)
    speeds = {}
    for a, path in vehicle_paths.items():
        tt = travel_times[a]
        if tt > 0:
            speeds[a] = (tt - waiting_times[a]) / tt
        else:
            speeds[a] = 0.0
            
    avg_speed = np.mean(list(speeds.values()))
    
    # 5. Collisions, Conflicts, and Near Misses
    veh_veh_collision_count = 0
    veh_ped_collision_count = 0
    veh_veh_conflict_count = 0
    veh_ped_conflict_count = 0
    veh_veh_near_miss_count = 0
    veh_ped_near_miss_count = 0
    
    max_t = max(len(p) for p in vehicle_paths.values()) if vehicle_paths else 0
    
    # Queue length tracking
    queue_lengths = []
    
    for t in range(max_t):
        # 5a. Queue Length at step t
        queued_vehs = 0
        for a, path in vehicle_paths.items():
            if t < len(path): # Still active
                if t > 0 and path[t] == path[t-1]:
                    queued_vehs += 1
        queue_lengths.append(queued_vehs)
        
        # Vehicle positions
        positions = {}
        for a, path in vehicle_paths.items():
            positions[a] = get_agent_pos_at_t(path, t)
            
        agent_ids = list(vehicle_paths.keys())
        
        # Vehicle-Vehicle conflict checks
        for i in range(len(agent_ids)):
            a1 = agent_ids[i]
            p1 = positions[a1]
            for j in range(i + 1, len(agent_ids)):
                a2 = agent_ids[j]
                p2 = positions[a2]
                
                # Exclude if both finished
                if t >= len(vehicle_paths[a1]) - 1 and t >= len(vehicle_paths[a2]) - 1:
                    continue
                    
                dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                if dist < 0.1:
                    veh_veh_collision_count += 1
                else:
                    if dist <= 1.5:
                        veh_veh_conflict_count += 1
                    if dist <= 1.0:
                        veh_veh_near_miss_count += 1
                        
        # Vehicle-Pedestrian conflict checks
        for a, path in vehicle_paths.items():
            p_veh = positions[a]
            if t >= len(path) - 1:
                continue
            for ped in pedestrian_paths:
                p_ped = get_agent_pos_at_t(ped['path'], t)
                if t >= len(ped['path']) - 1:
                    continue
                    
                dist = np.sqrt((p_veh[0] - p_ped[0])**2 + (p_veh[1] - p_ped[1])**2)
                if dist < 0.1:
                    veh_ped_collision_count += 1
                else:
                    # Vehicle-Pedestrian safety distance sum: R_veh = 0.75, R_ped = 0.25 -> 1.0
                    if dist <= 1.0:
                        veh_ped_conflict_count += 1
                    if dist <= 0.75:
                        veh_ped_near_miss_count += 1

    max_queue_length = max(queue_lengths) if queue_lengths else 0
    throughput = num_vehicles / max_t if max_t > 0 else 0.0
    
    # 6. Time-to-Collision (TTC) & Critical TTC
    veh_veh_ttc_list = []
    veh_ped_ttc_list = []
    
    for t in range(1, max_t):
        # Veh-Veh TTC
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
                
                if t >= len(path1) - 1 and t >= len(path2) - 1:
                    continue
                    
                # d_vv = 1.5
                ttc = calculate_pair_ttc(p1, v1, p2, v2, d=1.5)
                if ttc < float('inf'):
                    veh_veh_ttc_list.append(ttc)
                    
        # Veh-Ped TTC
        for a in vehicle_paths.keys():
            path1 = vehicle_paths[a]
            p1 = np.array(get_agent_pos_at_t(path1, t))
            p1_prev = np.array(get_agent_pos_at_t(path1, t-1))
            v1 = p1 - p1_prev
            
            if t >= len(path1) - 1:
                continue
                
            for ped in pedestrian_paths:
                p2 = np.array(get_agent_pos_at_t(ped['path'], t))
                p2_prev = np.array(get_agent_pos_at_t(ped['path'], t-1))
                v2 = p2 - p2_prev
                
                if t >= len(ped['path']) - 1:
                    continue
                    
                # d_vp = 1.0
                ttc = calculate_pair_ttc(p1, v1, p2, v2, d=1.0)
                if ttc < float('inf'):
                    veh_ped_ttc_list.append(ttc)
                        
    veh_veh_min_ttc = np.min(veh_veh_ttc_list) if veh_veh_ttc_list else 10.0
    veh_veh_avg_ttc = np.mean(veh_veh_ttc_list) if veh_veh_ttc_list else 10.0
    veh_veh_critical_ttc_events = sum(1 for val in veh_veh_ttc_list if val <= 1.5)
    
    veh_ped_min_ttc = np.min(veh_ped_ttc_list) if veh_ped_ttc_list else 10.0
    veh_ped_avg_ttc = np.mean(veh_ped_ttc_list) if veh_ped_ttc_list else 10.0
    veh_ped_critical_ttc_events = sum(1 for val in veh_ped_ttc_list if val <= 1.5)
    
    all_ttcs = veh_veh_ttc_list + veh_ped_ttc_list
    min_ttc = np.min(all_ttcs) if all_ttcs else 10.0
    avg_ttc = np.mean(all_ttcs) if all_ttcs else 10.0
    critical_ttc_events = sum(1 for val in all_ttcs if val <= 1.5)
    
    # 7. Post-Encroachment Time (PET) & Critical PET
    cell_occupancy = {} # (x, y) -> list of (time, agent_id)
    for a, path in vehicle_paths.items():
        for t, pos in enumerate(path):
            if t < len(path):
                if pos not in cell_occupancy:
                    cell_occupancy[pos] = []
                cell_occupancy[pos].append((t, f"veh_{a}"))
                
    for ped in pedestrian_paths:
        ped_id = ped['id']
        for t, pos in enumerate(ped['path']):
            if t < len(ped['path']):
                if pos not in cell_occupancy:
                    cell_occupancy[pos] = []
                cell_occupancy[pos].append((t, f"ped_{ped_id}"))
                
    veh_veh_pet_list = []
    veh_ped_pet_list = []
    
    for pos, occupancies in cell_occupancy.items():
        if len(occupancies) > 1:
            occupancies.sort()
            for k in range(len(occupancies) - 1):
                t1, a1 = occupancies[k]
                t2, a2 = occupancies[k+1]
                if a1 != a2:
                    pet = t2 - t1
                    if a1.startswith("veh_") and a2.startswith("veh_"):
                        veh_veh_pet_list.append(pet)
                    elif (a1.startswith("veh_") and a2.startswith("ped_")) or (a1.startswith("ped_") and a2.startswith("veh_")):
                        veh_ped_pet_list.append(pet)
                        
    veh_veh_min_pet = np.min(veh_veh_pet_list) if veh_veh_pet_list else 10.0
    veh_veh_avg_pet = np.mean(veh_veh_pet_list) if veh_veh_pet_list else 10.0
    veh_veh_critical_pet_events = sum(1 for val in veh_veh_pet_list if val <= 2.0)
    
    veh_ped_min_pet = np.min(veh_ped_pet_list) if veh_ped_pet_list else 10.0
    veh_ped_avg_pet = np.mean(veh_ped_pet_list) if veh_ped_pet_list else 10.0
    veh_ped_critical_pet_events = sum(1 for val in veh_ped_pet_list if val <= 2.0)
    
    all_pets = veh_veh_pet_list + veh_ped_pet_list
    min_pet = np.min(all_pets) if all_pets else 10.0
    avg_pet = np.mean(all_pets) if all_pets else 10.0
    critical_pet_events = sum(1 for val in all_pets if val <= 2.0)
    
    return {
        'avg_travel_time': float(avg_travel_time),
        'avg_waiting_time': float(avg_waiting_time),
        'avg_delay': float(avg_delay),
        'avg_speed': float(avg_speed),
        'veh_veh_collision_count': int(veh_veh_collision_count),
        'veh_ped_collision_count': int(veh_ped_collision_count),
        'collision_count': int(veh_veh_collision_count + veh_ped_collision_count),
        'veh_veh_conflict_count': int(veh_veh_conflict_count),
        'veh_ped_conflict_count': int(veh_ped_conflict_count),
        'conflict_count': int(veh_veh_conflict_count + veh_ped_conflict_count),
        'veh_veh_near_miss_count': int(veh_veh_near_miss_count),
        'veh_ped_near_miss_count': int(veh_ped_near_miss_count),
        'near_miss_count': int(veh_veh_near_miss_count + veh_ped_near_miss_count),
        'veh_veh_min_ttc': float(veh_veh_min_ttc),
        'veh_veh_avg_ttc': float(veh_veh_avg_ttc),
        'veh_ped_min_ttc': float(veh_ped_min_ttc),
        'veh_ped_avg_ttc': float(veh_ped_avg_ttc),
        'min_ttc': float(min_ttc),
        'avg_ttc': float(avg_ttc),
        'veh_veh_min_pet': float(veh_veh_min_pet),
        'veh_veh_avg_pet': float(veh_veh_avg_pet),
        'veh_ped_min_pet': float(veh_ped_min_pet),
        'veh_ped_avg_pet': float(veh_ped_avg_pet),
        'min_pet': float(min_pet),
        'avg_pet': float(avg_pet),
        'veh_veh_critical_ttc_events': int(veh_veh_critical_ttc_events),
        'veh_ped_critical_ttc_events': int(veh_ped_critical_ttc_events),
        'critical_ttc_events': int(critical_ttc_events),
        'veh_veh_critical_pet_events': int(veh_veh_critical_pet_events),
        'veh_ped_critical_pet_events': int(veh_ped_critical_pet_events),
        'critical_pet_events': int(critical_pet_events),
        'max_queue_length': int(max_queue_length),
        'throughput': float(throughput)
    }

def save_metrics_to_csv(metrics_dict, file_path):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    df = pd.DataFrame([metrics_dict])
    df.to_csv(file_path, index=False)
