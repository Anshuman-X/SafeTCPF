import os
import subprocess
import time
import sys
import yaml
import traci

class SumoEnvironment:
    def __init__(self, config_path="config/config.yaml"):
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.grid_width = self.config['simulation']['grid_width']
        self.grid_height = self.config['simulation']['grid_height']
        
        self.sumo_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sumo_files")
        os.makedirs(self.sumo_dir, exist_ok=True)
        
        # Paths to SUMO files (standalone demo)
        self.nod_xml = os.path.join(self.sumo_dir, "net.nod.xml")
        self.edg_xml = os.path.join(self.sumo_dir, "net.edg.xml")
        self.con_xml = os.path.join(self.sumo_dir, "net.con.xml")
        self.rou_xml = os.path.join(self.sumo_dir, "net.rou.xml")
        self.net_xml = os.path.join(self.sumo_dir, "net.net.xml")
        self.sumocfg = os.path.join(self.sumo_dir, "net.sumocfg")
        self.ped_xml = os.path.join(self.sumo_dir, "net.ped.xml")
        
        # Paths to active files (for TraCI control without pre-defined vehicle/person entities)
        self.active_rou_xml = os.path.join(self.sumo_dir, "active.rou.xml")
        self.active_ped_xml = os.path.join(self.sumo_dir, "active.ped.xml")
        self.active_sumocfg = os.path.join(self.sumo_dir, "active.sumocfg")
        self.view_xml = os.path.join(self.sumo_dir, "view.xml")
        
        self.generate_xml_files()
        self.compile_network()
        
    def generate_xml_files(self):
        # 1. Nodes XML
        nodes_content = """<nodes>
    <node id="center" x="0" y="0" type="priority"/>
    <node id="north" x="0" y="100" type="priority"/>
    <node id="south" x="0" y="-100" type="priority"/>
    <node id="east" x="100" y="0" type="priority"/>
    <node id="west" x="-100" y="0" type="priority"/>
</nodes>
"""
        with open(self.nod_xml, "w") as f:
            f.write(nodes_content)
            
        # 2. Edges XML
        edges_content = """<edges>
    <edge id="N_to_C" from="north" to="center" numLanes="2" speed="13.89"/>
    <edge id="C_to_S" from="center" to="south" numLanes="2" speed="13.89"/>
    <edge id="S_to_C" from="south" to="center" numLanes="2" speed="13.89"/>
    <edge id="C_to_N" from="center" to="north" numLanes="2" speed="13.89"/>
    <edge id="E_to_C" from="east" to="center" numLanes="2" speed="13.89"/>
    <edge id="C_to_W" from="center" to="west" numLanes="2" speed="13.89"/>
    <edge id="W_to_C" from="west" to="center" numLanes="2" speed="13.89"/>
    <edge id="C_to_E" from="center" to="east" numLanes="2" speed="13.89"/>
</edges>
"""
        with open(self.edg_xml, "w") as f:
            f.write(edges_content)
            
        # 3. Connections XML
        connections_content = """<connections>
    <connection from="N_to_C" to="C_to_S" fromLane="0" toLane="0"/>
    <connection from="N_to_C" to="C_to_W" fromLane="1" toLane="1"/>
    <connection from="S_to_C" to="C_to_N" fromLane="0" toLane="0"/>
    <connection from="S_to_C" to="C_to_E" fromLane="1" toLane="1"/>
    <connection from="E_to_C" to="C_to_W" fromLane="0" toLane="0"/>
    <connection from="E_to_C" to="C_to_S" fromLane="1" toLane="1"/>
    <connection from="W_to_C" to="C_to_E" fromLane="0" toLane="0"/>
    <connection from="W_to_C" to="C_to_N" fromLane="1" toLane="1"/>
</connections>
"""
        with open(self.con_xml, "w") as f:
            f.write(connections_content)
            
        # Common vTypes and Named routes (template used by both standalone and active)
        vtypes_routes_template = """    <vType id="car" vClass="passenger" length="5.0" width="2.0" maxSpeed="13.89" accel="2.6" decel="4.5" sigma="0.5"/>
    <vType id="motorcycle" vClass="motorcycle" length="2.0" width="0.8" maxSpeed="16.67" accel="4.0" decel="6.0" sigma="0.7"/>
    <vType id="autorickshaw" vClass="taxi" length="3.2" width="1.4" maxSpeed="10.0" accel="1.5" decel="3.5" sigma="0.6"/>
    <vType id="bus" vClass="bus" length="12.0" width="2.5" maxSpeed="11.11" accel="1.2" decel="2.5" sigma="0.4"/>
    <vType id="bicycle" vClass="bicycle" length="1.6" width="0.6" maxSpeed="5.56" accel="1.0" decel="2.0" sigma="0.8"/>
    <vType id="ped" vClass="pedestrian" length="0.4" width="0.5" maxSpeed="1.5"/>

    <!-- Named routes -->
    <route id="r_dummy" edges="N_to_C C_to_S"/>
    <route id="r_NS"    edges="N_to_C C_to_S"/>
    <route id="r_SN"    edges="S_to_C C_to_N"/>
    <route id="r_EW"    edges="E_to_C C_to_W"/>
    <route id="r_WE"    edges="W_to_C C_to_E"/>
    <route id="r_NW"    edges="N_to_C C_to_W"/>
    <route id="r_SE"    edges="S_to_C C_to_E"/>"""

        # Read parameters from traffic_generation
        gen_cfg = self.config.get('traffic_generation', {})
        seed = gen_cfg.get('seed', 42)
        duration = gen_cfg.get('duration_steps', 1000)
        t_density = gen_cfg.get('selected_traffic_density', 'medium')
        p_density = gen_cfg.get('selected_pedestrian_density', 'medium')

        veh_spawn_rate = gen_cfg.get('vehicle_spawn_rates', {}).get(t_density, 0.20)
        ped_spawn_rate = gen_cfg.get('pedestrian_spawn_rates', {}).get(p_density, 0.15)

        proportions = gen_cfg.get('proportions', {
            'car': 0.40,
            'motorcycle': 0.35,
            'autorickshaw': 0.10,
            'bus': 0.05,
            'bicycle': 0.10
        })

        import random
        rng = random.Random(seed)

        # 4. Standalone Routes XML (contains concrete vehicle definitions for standalone sumo-gui mode)
        routes_list = ["r_NS", "r_SN", "r_EW", "r_WE", "r_NW", "r_SE"]
        standalone_flows = []
        for route in routes_list:
            for vtype, proportion in proportions.items():
                # Spawning probability per second for this specific route and vehicle type
                flow_prob = veh_spawn_rate * (1.0 / len(routes_list)) * proportion
                if flow_prob > 0:
                    flow_id = f"flow_{vtype}_{route}"
                    standalone_flows.append(
                        f'    <flow id="{flow_id}" type="{vtype}" route="{route}" begin="0" end="{duration}" '
                        f'probability="{flow_prob:.6f}" departLane="best" departSpeed="max"/>'
                    )

        vehicles_xml_block = "\n".join(standalone_flows)
        standalone_routes_content = f"""<routes>
{vtypes_routes_template}

    <!-- Config-driven mixed vehicles continuously spawned as flows (reproducible seed={seed}) -->
{vehicles_xml_block}
</routes>
"""
        with open(self.rou_xml, "w") as f:
            f.write(standalone_routes_content)

        # 5. Standalone Pedestrians XML (contains concrete pedestrian definitions with walk stages)
        ped_walks = [
            "N_to_C C_to_S",
            "S_to_C C_to_N",
            "E_to_C C_to_W",
            "W_to_C C_to_E",
            "N_to_C C_to_W",
            "S_to_C C_to_E"
        ]
        standalone_ped_flows = []
        for i, walk in enumerate(ped_walks):
            # Spawning probability per second for this specific walk route
            ped_flow_prob = ped_spawn_rate / len(ped_walks)
            if ped_flow_prob > 0:
                pflow_id = f"pflow_{i}"
                standalone_ped_flows.append(
                    f'    <personFlow id="{pflow_id}" type="ped" begin="0" end="{duration}" probability="{ped_flow_prob:.6f}">\n'
                    f'        <walk edges="{walk}"/>\n'
                    f'    </personFlow>'
                )

        pedestrians_xml_block = "\n".join(standalone_ped_flows)
        standalone_ped_content = f"""<routes>
    <!-- Config-driven dynamic pedestrians continuously spawned as personFlows on sidewalks and crossings -->
{pedestrians_xml_block}
</routes>
"""
        with open(self.ped_xml, "w") as f:
            f.write(standalone_ped_content)

        # 6. Active Routes XML (clean version without concrete vehicles, used by TraCI)
        active_routes_content = f"""<routes>
{vtypes_routes_template}
</routes>
"""
        with open(self.active_rou_xml, "w") as f:
            f.write(active_routes_content)

        # 7. Active Pedestrians XML (clean version without concrete pedestrians, used by TraCI)
        active_ped_content = """<routes>
    <!-- Pedestrians routing headers only -->
</routes>
"""
        with open(self.active_ped_xml, "w") as f:
            f.write(active_ped_content)

        # 8. GUI View Settings XML (makes vehicles and pedestrians highly visible and beautiful)
        view_content = """<viewsettings>
    <viewport zoom="400" x="100" y="100"/>
    <delay value="150"/>
    <scheme name="real world"/>
    <persons personMode="0" personSize="2.2" personColor="yellow"/>
    <vehicles vehicleMode="0" vehicleSize="1.3"/>
</viewsettings>
"""
        with open(self.view_xml, "w") as f:
            f.write(view_content)

        # 9. Standalone SUMO Config
        sumocfg_content = f"""<configuration>
    <input>
        <net-file value="net.net.xml"/>
        <route-files value="net.rou.xml,net.ped.xml"/>
        <gui-settings-file value="view.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{duration}"/>
        <step-length value="1.0"/>
    </time>
</configuration>
"""
        with open(self.sumocfg, "w") as f:
            f.write(sumocfg_content)

        # 10. Active SUMO Config (used by TraCI)
        active_sumocfg_content = f"""<configuration>
    <input>
        <net-file value="net.net.xml"/>
        <route-files value="active.rou.xml,active.ped.xml"/>
        <gui-settings-file value="view.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="{duration}"/>
        <step-length value="1.0"/>
    </time>
</configuration>
"""
        with open(self.active_sumocfg, "w") as f:
            f.write(active_sumocfg_content)

    def compile_network(self):
        # Compile network using netconvert
        # --sidewalks.guess  → adds a sidewalk lane to every edge (pedestrians)
        # --crossings.guess  → adds pedestrian crossings at junctions
        # Connection file omitted: sidewalks shift lane indices (old lane 0/1
        # become 1/2), so we let netconvert auto-generate connections.
        cmd = [
            "netconvert",
            f"--node-files={self.nod_xml}",
            f"--edge-files={self.edg_xml}",
            f"--output-file={self.net_xml}",
            "--sidewalks.guess",
            "--crossings.guess"
        ]
        try:
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
            with open(os.path.join(os.path.dirname(self.nod_xml), "../logs/build_log.txt"), "a") as log:
                log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] netconvert run successfully.\n")
        except subprocess.CalledProcessError as e:
            print(f"Error compiling network: {e.stderr.decode('utf-8')}", file=sys.stderr)
            raise e

    def grid_to_sumo(self, x, y):
        """Convert grid coordinates to SUMO world coordinates.

        The SUMO network uses netOffset=(100,100), so world coords span
        [0, 200] × [0, 200] with the center junction at (100, 100).

        Grid layout (20 wide × 24 tall):
          Vertical lanes:   cols  8, 9  (southbound)  /  10, 11 (northbound)
          Horizontal lanes: rows 10, 11 (eastbound)   /  12, 13 (westbound)
          Intersection:     x ∈ [8..11], y ∈ [10..13]

        Piecewise linear mapping ensures vehicles and pedestrians are aligned
        accurately with the physical layout of lanes, sidewalks, and crossings.
        """
        # Map x to X coordinate
        if x <= 7:
            X = x * (92.60 / 7.0)
        elif x >= 12:
            X = 107.40 + (x - 12) * ((200.00 - 107.40) / 7.0)
        else:
            X_map = {8: 95.20, 9: 98.40, 10: 101.60, 11: 104.80}
            X = X_map[x]

        # Map y to Y coordinate
        if y <= 9:
            Y = y * (92.60 / 9.0)
        elif y >= 14:
            Y = 107.40 + (y - 14) * ((200.00 - 107.40) / 9.0)
        else:
            Y_map = {10: 95.20, 11: 98.40, 12: 101.60, 13: 104.80}
            Y = Y_map[y]

        return X, Y

    def get_edge_id(self, x, y):
        """Map a grid position to the most likely SUMO edge ID.

        Used as a *hint* for moveToXY(keepRoute=2) — SUMO will snap to the
        nearest valid lane regardless, so this does not need to be perfect
        for cells inside the intersection.
        """
        if x in [8, 9]:       # Southbound columns
            return "N_to_C" if y >= 14 else "C_to_S"
        elif x in [10, 11]:   # Northbound columns
            return "S_to_C" if y <= 9 else "C_to_N"
        elif y in [10, 11]:   # Eastbound rows
            return "W_to_C" if x <= 7 else "C_to_E"
        elif y in [12, 13]:   # Westbound rows
            return "E_to_C" if x >= 12 else "C_to_W"
        return "N_to_C"

    def get_lane_index(self, x, y):
        """Get the appropriate vehicle lane index.

        Since lane 0 is the pedestrian sidewalk, vehicle lanes are indices 1 and 2.
        """
        if x in [8, 9]:
            return 1 if x == 8 else 2
        elif x in [10, 11]:
            return 2 if x == 10 else 1
        elif y in [10, 11]:
            return 1 if y == 10 else 2
        elif y in [12, 13]:
            return 2 if y == 12 else 1
        return 1

    def start_simulation(self, gui=False):
        # Choose sumo binary
        sumo_binary = "sumo-gui" if gui else "sumo"
        cmd = [sumo_binary, "-c", self.active_sumocfg, "--no-warnings", "--step-length", "1.0"]
        if gui:
            # Start simulation automatically and add 500ms delay per step.
            # Do NOT add --quit-on-end so the GUI remains open for manual closing.
            cmd.extend(["--start", "--delay", "500"])
        traci.start(cmd)
        
    def step(self):
        traci.simulationStep()
        
    def stop_simulation(self):
        try:
            traci.close()
        except Exception:
            pass
            
    def get_route_id(self, x, y):
        """Map starting grid positions to correct named routes."""
        if x in [8, 9]:
            return "r_NS"
        elif x in [10, 11]:
            return "r_SN"
        elif y in [10, 11]:
            return "r_WE"
        elif y in [12, 13]:
            return "r_EW"
        return "r_dummy"

    def spawn_vehicle(self, veh_id, x, y, typeID="car"):
        """Spawn a vehicle at the given grid position via TraCI."""
        X, Y = self.grid_to_sumo(x, y)
        edge_id = self.get_edge_id(x, y)
        lane_idx = self.get_lane_index(x, y)
        route_id = self.get_route_id(x, y)
        try:
            traci.vehicle.add(veh_id, route_id, typeID=typeID)
            traci.vehicle.moveToXY(veh_id, edge_id, lane_idx, X, Y, angle=0, keepRoute=2)
        except traci.exceptions.TraCIException as e:
            print(f"  Warning: Could not spawn vehicle {veh_id}: {e}")
        
    def move_vehicle(self, veh_id, x, y, angle=0):
        X, Y = self.grid_to_sumo(x, y)
        edge_id = self.get_edge_id(x, y)
        lane_idx = self.get_lane_index(x, y)
        try:
            traci.vehicle.moveToXY(veh_id, edge_id, lane_idx, X, Y, angle=angle, keepRoute=2)
        except traci.exceptions.TraCIException:
            # Vehicle might have arrived or not spawnable
            pass
            
    def spawn_pedestrian(self, ped_id: str, x: int, y: int) -> None:
        """Spawn a pedestrian at the given grid position via TraCI.

        Pedestrians are placed on a sidewalk edge at its midpoint with a long
        waiting stage, then immediately snapped to their real grid position.
        keepRoute=0 forces SUMO to find the nearest pedestrian-accessible lane
        (sidewalk, crossing, or walking area) so they never appear in grass or
        inside buildings.
        """
        try:
            # Place on a default sidewalk edge at the midpoint.
            # The initial position does not matter because we immediately snap
            # with moveToXY.  Edge N_to_C lane 0 is the guessed sidewalk.
            edge_id = "N_to_C"
            pos = 45.0  # roughly mid-edge (edge length ≈ 89.6 m)
            traci.person.add(ped_id, edge_id, pos, typeID="ped")
            # Person MUST have at least one plan stage or SUMO removes them.
            traci.person.appendWaitingStage(ped_id, 1000.0, "planner_controlled")
            # Snap to the real grid position on the nearest walkable lane.
            # keepRoute=0 allows any edge and replaces the current route with
            # the edge found, preventing out-of-network placement.
            X, Y = self.grid_to_sumo(x, y)
            traci.person.moveToXY(ped_id, "", X, Y, angle=0, keepRoute=0)
        except traci.exceptions.TraCIException as e:
            print(f"  Warning: Could not spawn pedestrian {ped_id}: {e}")
        
    def move_pedestrian(self, ped_id: str, x: int, y: int) -> None:
        """Move a pedestrian to a new grid position via TraCI.

        keepRoute=0 forces SUMO to snap to the nearest walkable lane so the
        pedestrian stays on valid infrastructure at all times.
        """
        X, Y = self.grid_to_sumo(x, y)
        try:
            traci.person.moveToXY(ped_id, "", X, Y, angle=0, keepRoute=0)
        except traci.exceptions.TraCIException:
            pass

    def spawn_background_vehicles(self, density: str = "medium", step: int = 0, seed: int = None) -> None:
        """Dynamically spawn background traffic at boundary edges based on density.

        A gap-check prevents spawning a new vehicle on an edge that already has
        another vehicle within 15 m of the entry point, eliminating overlap at spawn.
        """
        bg_traffic_config = self.config.get('simulation', {}).get('background_traffic', {})
        if not bg_traffic_config.get('enabled', True):
            return

        import random
        # Ensure unique but reproducible spawns at each step
        if seed is not None:
            random.seed(seed + step)

        probs = bg_traffic_config.get('probabilities', {
            'low': 0.05,
            'medium': 0.15,
            'high': 0.30
        })

        prob = probs.get(density, 0.15)

        # Check if we spawn a vehicle in this step
        if random.random() < prob:
            routes = ["r_NS", "r_SN", "r_EW", "r_WE", "r_NW", "r_SE"]
            route_id = random.choice(routes)
            veh_id = f"bg_{step}_{random.randint(0, 1000)}"

            # Select vehicle type dynamically (heterogeneous mixed traffic)
            proportions = self.config.get('simulation', {}).get('mixed_traffic', {}).get('proportions', {
                'car': 0.45,
                'motorcycle': 0.30,
                'autorickshaw': 0.15,
                'bus': 0.05,
                'bicycle': 0.05
            })
            vtypes = list(proportions.keys())
            weights = list(proportions.values())
            type_id = random.choices(vtypes, weights=weights)[0]

            # Gap-check: map route to its entry edge and verify no vehicle is
            # within 15 m of the edge start before spawning.
            route_to_edge = {
                "r_NS": "N_to_C", "r_NW": "N_to_C",
                "r_SN": "S_to_C", "r_SE": "S_to_C",
                "r_EW": "E_to_C",
                "r_WE": "W_to_C",
            }
            entry_edge = route_to_edge.get(route_id, "N_to_C")
            gap_clear = True
            try:
                vehs_on_edge = traci.edge.getLastStepVehicleIDs(entry_edge)
                for v in vehs_on_edge:
                    if traci.vehicle.getLanePosition(v) < 15.0:
                        gap_clear = False
                        break
            except Exception:
                gap_clear = True  # if TraCI query fails, allow spawn

            if gap_clear:
                try:
                    traci.vehicle.add(veh_id, route_id, typeID=type_id)
                except traci.exceptions.TraCIException:
                    pass

    def update_gui_overlays(self, active_veh_paths, pedestrians, step_idx):
        """Updates colors in SUMO GUI based on risk coloring and draws active planned path POIs (Objective 20)."""
        import numpy as np
        
        # Clean up POIs from previous steps
        try:
            poi_ids = traci.poi.getIDList()
            for pid in poi_ids:
                if pid.startswith("poi_"):
                    traci.poi.remove(pid)
        except Exception:
            pass
            
        # Draw remaining path POIs for active planning vehicles
        for a, path in active_veh_paths.items():
            veh_id = f"veh_{a}"
            try:
                if veh_id in traci.vehicle.getIDList():
                    poi_color = (0, 191, 255, 255) # cyan
                    
                    # Draw remaining steps in path (only up to next 10 cells to avoid overlay clutter)
                    for t_idx in range(step_idx, min(len(path), step_idx + 10)):
                        pos = path[t_idx]
                        X, Y = self.grid_to_sumo(pos[0], pos[1])
                        poi_id = f"poi_{veh_id}_{t_idx}"
                        traci.poi.add(poi_id, X, Y, poi_color, poiType="path", layer=1)
            except Exception:
                pass

        # Color vehicles according to real-time risk (TTC to other vehicles/pedestrians)
        try:
            all_veh_ids = traci.vehicle.getIDList()
        except Exception:
            return
            
        for veh_id in all_veh_ids:
            try:
                p1 = np.array(traci.vehicle.getPosition(veh_id))
                speed1 = traci.vehicle.getSpeed(veh_id)
                angle = traci.vehicle.getAngle(veh_id)
                # Convert angle to vector (0 is North, 90 is East)
                rad = np.deg2rad(90 - angle)
                v1 = np.array([np.cos(rad), np.sin(rad)]) * speed1
                
                min_ttc = float('inf')
                
                # Check TTC with other vehicles
                for other_id in all_veh_ids:
                    if other_id == veh_id:
                        continue
                    p2 = np.array(traci.vehicle.getPosition(other_id))
                    speed2 = traci.vehicle.getSpeed(other_id)
                    angle2 = traci.vehicle.getAngle(other_id)
                    rad2 = np.deg2rad(90 - angle2)
                    v2 = np.array([np.cos(rad2), np.sin(rad2)]) * speed2
                    
                    from metrics.safety_metrics import calculate_pair_ttc
                    ttc = calculate_pair_ttc(p1, v1, p2, v2, d=1.5)
                    if ttc >= 0 and ttc < min_ttc:
                        min_ttc = ttc
                        
                # Check TTC with pedestrians
                try:
                    ped_ids = traci.person.getIDList()
                except Exception:
                    ped_ids = []
                    
                for ped_id in ped_ids:
                    p2 = np.array(traci.person.getPosition(ped_id))
                    speed2 = traci.person.getSpeed(ped_id)
                    angle2 = traci.person.getAngle(ped_id)
                    rad2 = np.deg2rad(90 - angle2)
                    v2 = np.array([np.cos(rad2), np.sin(rad2)]) * speed2
                    
                    from metrics.safety_metrics import calculate_pair_ttc
                    ttc = calculate_pair_ttc(p1, v1, p2, v2, d=1.0)
                    if ttc >= 0 and ttc < min_ttc:
                        min_ttc = ttc
                
                # Risk coloring: Green (Safe), Yellow (Potential), Orange (Near), Red (Critical)
                if min_ttc > 3.0:
                    color = (0, 255, 0, 255) # Green
                elif min_ttc > 1.5:
                    color = (255, 255, 0, 255) # Yellow
                elif min_ttc > 0.5:
                    color = (255, 128, 0, 255) # Orange
                else:
                    color = (255, 0, 0, 255) # Red
                    
                traci.vehicle.setColor(veh_id, color)
            except Exception:
                pass

    def sumo_to_grid(self, X, Y):
        """Convert SUMO coordinates back to grid coordinates."""
        x = int(round(X * (19.0 / 200.0)))
        y = int(round(Y * (23.0 / 200.0)))
        x = max(0, min(self.grid_width - 1, x))
        y = max(0, min(self.grid_height - 1, y))
        return x, y

