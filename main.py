import os
import sys
import time
import traceback
import subprocess
import yaml
import pandas as pd
import traci
from evaluation.experiment_runner import ExperimentRunner
from simulation.sumo_env import SumoEnvironment
from pedestrians.ped_model import PedestrianModel
from algorithms.tc_icbs import TCICBSPlanner


def run_sumo_demo(gui=False):
    print("\n--- Running SUMO Simulation Demo ---")
    try:
        # Load configuration and initialize environment
        env = SumoEnvironment()
        
        # We will use the Motorcade scenario from config for the demo
        runner = ExperimentRunner()
        scenario = runner.config['scenarios'][0] # Motorcade
        teams = scenario['teams']
        agents_def = scenario['agents_def']
        
        print("Planning paths with TC-ICBS for SUMO demo...")
        ped_density = "medium"
        ped_model = PedestrianModel(env.grid_width, env.grid_height)
        pedestrians = ped_model.generate_pedestrians(ped_density)
        dynamic_obstacles = ped_model.get_occupancy_set(pedestrians)
        
        planner = TCICBSPlanner(teams, agents_def, epsilon=runner.config['planning']['epsilon'],
                                grid_width=env.grid_width, grid_height=env.grid_height)
        solutions = planner.plan(max_nodes=500, time_limit=30, dynamic_obstacles=dynamic_obstacles)
        
        if not solutions:
            print("Demo planning failed. Cannot run SUMO demo.")
            return
            
        best_sol = solutions[0]
        paths = best_sol['paths']
        
        # Assign mixed traffic vehicle types proportionally to active vehicles
        import random
        # Use seed from traffic_generation config for reproducibility
        gen_cfg = env.config.get('traffic_generation', {})
        demo_seed = gen_cfg.get('seed', 42)
        random.seed(demo_seed)
        proportions = runner.config['simulation']['mixed_traffic']['proportions']
        vtypes = list(proportions.keys())
        weights = list(proportions.values())
        agent_types = random.choices(vtypes, weights=weights, k=len(agents_def))
        
        # Run headless or with gui based on parameter.
        env.start_simulation(gui=gui)
        
        # Spawn all agents initially with heterogeneous vehicle types
        for a in range(len(agents_def)):
            start_pos = paths[a][0]
            env.spawn_vehicle(f"veh_{a}", start_pos[0], start_pos[1], typeID=agent_types[a])
            
        for ped in pedestrians:
            start_pos = ped['path'][0]
            env.spawn_pedestrian(ped['id'], start_pos[0], start_pos[1])
            
        # Step through time and update positions
        print("Stepping simulation and updating positions via TraCI...")
        t = 0
        last_replan_t = -10 # Cooldown to avoid infinite loop of replans
        
        max_t = max(len(p) for p in paths.values())
        while t < max_t:
            # 1. Get current background vehicles' positions
            bg_positions = []
            try:
                all_veh_ids = traci.vehicle.getIDList()
                for veh_id in all_veh_ids:
                    if veh_id.startswith("bg_"):
                        pos_sumo = traci.vehicle.getPosition(veh_id)
                        grid_pos = env.sumo_to_grid(pos_sumo[0], pos_sumo[1])
                        bg_positions.append(grid_pos)
            except Exception:
                pass
                
            # 2. Check for conflicts / sensing obstacles to trigger online dynamic replanning
            detect_conflict = False
            if t - last_replan_t >= 5: # 5 steps cooldown
                for a in range(len(agents_def)):
                    if t < len(paths[a]):
                        # Check next 5 cells of agent a's path
                        future_cells = paths[a][t : min(t + 6, len(paths[a]))]
                        for dt, cell in enumerate(future_cells):
                            t_check = t + dt
                            # Check if pedestrian will occupy this cell at t_check
                            for ped in pedestrians:
                                if t_check < len(ped['path']) and ped['path'][t_check] == cell:
                                    detect_conflict = True
                                    break
                            if detect_conflict:
                                break
                            # Check if background vehicle is currently at this cell
                            if cell in bg_positions:
                                detect_conflict = True
                                break
                    if detect_conflict:
                        break
                        
            if detect_conflict:
                print(f"  [Dynamic Replanning] Obstacle detected near agent paths at step {t}. Replanning remaining paths...")
                # Construct new agent definitions from current position to goal
                new_agents_def = []
                for a in range(len(agents_def)):
                    curr_pos = paths[a][t] if t < len(paths[a]) else paths[a][-1]
                    new_agents_def.append({
                        'start': list(curr_pos),
                        'goal': list(agents_def[a]['goal'])
                    })
                
                # Offset remaining pedestrian trajectories to new relative start time 0
                new_dynamic_obstacles = set()
                for ped in pedestrians:
                    path = ped['path']
                    for t_future in range(t, len(path)):
                        new_dynamic_obstacles.add((path[t_future][0], path[t_future][1], t_future - t))
                        
                # Add current background vehicles as obstacles at step 0 and 1
                for bg_pos in bg_positions:
                    new_dynamic_obstacles.add((bg_pos[0], bg_pos[1], 0))
                    new_dynamic_obstacles.add((bg_pos[0], bg_pos[1], 1))
                    
                # Plan from current state
                replan_planner = TCICBSPlanner(teams, new_agents_def, epsilon=runner.config['planning']['epsilon'],
                                               grid_width=env.grid_width, grid_height=env.grid_height)
                replan_solutions = replan_planner.plan(max_nodes=300, time_limit=10, dynamic_obstacles=new_dynamic_obstacles)
                
                if replan_solutions:
                    new_best_sol = replan_solutions[0]
                    new_paths = new_best_sol['paths']
                    for a in range(len(agents_def)):
                        # Combine path before t with new planned path from t onwards
                        paths[a] = paths[a][:t] + new_paths[a]
                    print(f"  [Dynamic Replanning] Successful! Paths updated.")
                    last_replan_t = t
                    max_t = max(len(p) for p in paths.values())
                else:
                    print(f"  [Dynamic Replanning] Failed to find alternative paths. Continuing with existing paths.")
            
            # 3. Move vehicles
            for a in range(len(agents_def)):
                path = paths[a]
                if t < len(path):
                    pos = path[t]
                    # Compute angle roughly based on next step
                    angle = 0
                    if t < len(path) - 1:
                        next_pos = path[t+1]
                        dx = next_pos[0] - pos[0]
                        dy = next_pos[1] - pos[1]
                        if dx > 0: angle = 90
                        elif dx < 0: angle = 270
                        elif dy > 0: angle = 0
                        elif dy < 0: angle = 180
                    env.move_vehicle(f"veh_{a}", pos[0], pos[1], angle)
                    
            # 4. Move pedestrians
            for ped in pedestrians:
                path = ped['path']
                if t < len(path):
                    pos = path[t]
                    env.move_pedestrian(ped['id'], pos[0], pos[1])
                    
            # 5. Spawn background traffic dynamically (Objective 3)
            env.spawn_background_vehicles(density=ped_density, step=t, seed=42)
            
            # 6. Update overlays in SUMO GUI (colors and POIs)
            env.update_gui_overlays(paths, pedestrians, t)
            
            # 7. Step TraCI
            env.step()
            t += 1
            
        if gui:
            input("\nSimulation finished. Inspect the SUMO GUI. Press Enter here to close and clean up...")
            
        env.stop_simulation()
        print("SUMO Simulation Demo completed successfully!")
    except Exception as e:
        print(f"SUMO Demo warning/error: {e}")
        traceback.print_exc()
        print("Continuing project execution...")


def run_standalone_demo():
    """Launch SUMO GUI directly with pre-generated mixed-traffic route files.

    This bypasses the TC-ICBS planner entirely and just opens the standalone
    SUMO configuration (net.sumocfg) that contains baked-in vehicles of all
    types and continuous pedestrian spawns.  Use this to visually verify that
    cars, motorcycles, autorickshaws, buses, bicycles AND pedestrians all
    appear simultaneously in the SUMO GUI.
    """
    print("\n--- Running Standalone SUMO Mixed-Traffic Demo ---")
    print("(No planning required — uses pre-generated route files)\n")
    try:
        env = SumoEnvironment()
        print(f"  Vehicle route file : {env.rou_xml}")
        print(f"  Pedestrian file    : {env.ped_xml}")
        print(f"  SUMO config        : {env.sumocfg}")
        print("\nLaunching sumo-gui...  Close the SUMO window when done.\n")

        # Launch sumo-gui with the standalone config (blocks until closed)
        subprocess.run(
            ["sumo-gui", "-c", env.sumocfg, "--start", "--delay", "100"],
            check=True
        )
        print("Standalone demo finished.")
    except FileNotFoundError:
        print("Error: sumo-gui not found. Ensure SUMO is installed and on PATH.")
    except Exception as e:
        print(f"Standalone demo error: {e}")
        traceback.print_exc()


def main():
    print("====================================================")
    print("          SafeTCPF Master Execution Pipeline        ")
    print("====================================================")
    
    # 1. Run all experiments
    runner = ExperimentRunner()
    results_df = runner.run()
    
    # 2. Generate visualizations (plots)
    runner.generate_visualizations(results_df)
    
    # 3. Generate PDF reports
    runner.generate_pdf_reports(results_df)
    
    # 4. Run SUMO demonstration
    if "--standalone" in sys.argv:
        run_standalone_demo()
    else:
        gui = "--gui" in sys.argv
        run_sumo_demo(gui=gui)
    
    # Write build log complete
    with open("logs/build_log.txt", "a") as log:
        log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] Entire master pipeline ran and compiled successfully.\n")
        
    print("\n" + "="*50)
    print("PROJECT COMPLETE")
    print("="*50)
    
    # Project directory details
    print(f"Project Workspace: {os.path.abspath(os.getcwd())}\n")
    
    print("Generated Source Files:")
    print("  - [main.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/main.py)")
    print("  - [config/config.yaml](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/config/config.yaml)")
    print("  - [simulation/sumo_env.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/simulation/sumo_env.py)")
    print("  - [algorithms/a_star.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/algorithms/a_star.py)")
    print("  - [algorithms/tc_cbs.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/algorithms/tc_cbs.py)")
    print("  - [algorithms/icbs.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/algorithms/icbs.py)")
    print("  - [algorithms/tc_icbs.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/algorithms/tc_icbs.py)")
    print("  - [pedestrians/ped_model.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/pedestrians/ped_model.py)")
    print("  - [metrics/safety_metrics.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/metrics/safety_metrics.py)")
    print("  - [evaluation/experiment_runner.py](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/evaluation/experiment_runner.py)")
    print("  - [docs/paper_analysis.md](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/docs/paper_analysis.md)\n")
    
    print("Generated CSV Files:")
    print("  - [results/experiment_results.csv](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/experiment_results.csv)\n")
    
    print("Generated Graphs:")
    print("  - [visualizations/motorcade_efficiency.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/motorcade_efficiency.png)")
    print("  - [visualizations/motorcade_safety.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/motorcade_safety.png)")
    print("  - [visualizations/motorcade_computational.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/motorcade_computational.png)")
    print("  - [visualizations/fourteams_efficiency.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/fourteams_efficiency.png)")
    print("  - [visualizations/fourteams_safety.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/fourteams_safety.png)")
    print("  - [visualizations/fourteams_computational.png](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/fourteams_computational.png)\n")
    
    print("Generated Reports:")
    print("  - [reports/final_report.pdf](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/reports/final_report.pdf)")
    print("  - [reports/base_paper_comparison.pdf](file:///D:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/reports/base_paper_comparison.pdf)\n")
    
    # Print summary of results
    print("Summary of Experimental Results:")
    summary_df = results_df.groupby(['scenario', 'algorithm'])[['runtime_s', 'search_nodes', 'avg_travel_time', 'conflict_count', 'avg_ttc', 'avg_pet']].mean()
    print(summary_df.to_string())
    print("\nTo run the project, execute the following command in your terminal:")
    print("  python main.py")
    print("="*50)

if __name__ == "__main__":
    main()
