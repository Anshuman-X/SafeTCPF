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
    <vType id="ped" vClass="pedestrian" length="0.4" width="0.5" maxSpeed="1.5"/>

    <!-- Named routes -->
    <route id="r_dummy" edges="N_to_C C_to_S"/>
    <route id="r_NS"    edges="N_to_C C_to_S"/>
    <route id="r_SN"    edges="S_to_C C_to_N"/>
    <route id="r_EW"    edges="E_to_C C_to_W"/>
    <route id="r_WE"    edges="W_to_C C_to_E"/>
    <route id="r_NW"    edges="N_to_C C_to_W"/>
    <route id="r_SE"    edges="S_to_C C_to_E"/>"""

        # 4. Standalone Routes XML (contains concrete vehicle definitions for standalone sumo-gui mode)
        standalone_routes_content = f"""<routes>
{vtypes_routes_template}

    <!-- Concrete vehicles for standalone demo (sorted by depart time) -->
    <vehicle id="veh_ns0" type="car" route="r_NS" depart="0.0"/>
    <vehicle id="veh_sn0" type="car" route="r_SN" depart="0.0"/>
    <vehicle id="veh_ew0" type="car" route="r_EW" depart="2.0"/>
    <vehicle id="veh_we0" type="car" route="r_WE" depart="2.0"/>
    <vehicle id="veh_nw0" type="car" route="r_NW" depart="5.0"/>
    <vehicle id="veh_se0" type="car" route="r_SE" depart="5.0"/>
    <vehicle id="veh_ns1" type="car" route="r_NS" depart="10.0"/>
    <vehicle id="veh_sn1" type="car" route="r_SN" depart="10.0"/>
    <vehicle id="veh_ew1" type="car" route="r_EW" depart="12.0"/>
    <vehicle id="veh_we1" type="car" route="r_WE" depart="12.0"/>
    <vehicle id="veh_nw1" type="car" route="r_NW" depart="15.0"/>
    <vehicle id="veh_se1" type="car" route="r_SE" depart="15.0"/>
    <vehicle id="veh_ns2" type="car" route="r_NS" depart="20.0"/>
    <vehicle id="veh_sn2" type="car" route="r_SN" depart="20.0"/>
    <vehicle id="veh_ew2" type="car" route="r_EW" depart="22.0"/>
    <vehicle id="veh_we2" type="car" route="r_WE" depart="22.0"/>
</routes>
"""
        with open(self.rou_xml, "w") as f:
            f.write(standalone_routes_content)

        # 5. Standalone Pedestrians XML (contains concrete pedestrian definitions with walk stages)
        standalone_ped_content = """<routes>
    <!-- Concrete pedestrians for standalone demo (sorted by depart time) -->
    <person id="ped_0" depart="0.0" type="ped">
        <walk edges="N_to_C C_to_S"/>
    </person>
    <person id="ped_1" depart="2.0" type="ped">
        <walk edges="S_to_C C_to_N"/>
    </person>
    <person id="ped_2" depart="4.0" type="ped">
        <walk edges="E_to_C C_to_W"/>
    </person>
    <person id="ped_3" depart="6.0" type="ped">
        <walk edges="W_to_C C_to_E"/>
    </person>
    <person id="ped_4" depart="8.0" type="ped">
        <walk edges="N_to_C C_to_E"/>
    </person>
    <person id="ped_5" depart="10.0" type="ped">
        <walk edges="S_to_C C_to_W"/>
    </person>
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
        <end value="1000"/>
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
        <end value="1000"/>
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

        Strategy:
          - For vehicles on vertical lanes, use the exact SUMO X coordinate
            of the lane and interpolate Y linearly across the grid height.
          - For vehicles on horizontal lanes, use the exact SUMO Y coordinate
            of the lane and interpolate X linearly across the grid width.
          - For other positions (pedestrian crosswalks, off-road), use pure
            linear interpolation on both axes.
        """
        # Exact SUMO X positions of each vertical-lane column
        LANE_X = {8: 95.20, 9: 98.40, 10: 101.60, 11: 104.80}
        # Exact SUMO Y positions of each horizontal-lane row
        LANE_Y = {10: 95.20, 11: 98.40, 12: 101.60, 13: 104.80}

        if x in LANE_X:
            X = LANE_X[x]
            Y = y * (200.0 / 23.0)
        elif y in LANE_Y:
            X = x * (200.0 / 19.0)
            Y = LANE_Y[y]
        else:
            X = x * (200.0 / 19.0)
            Y = y * (200.0 / 23.0)

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
        if x in [8, 9]:
            return 0 if x == 8 else 1
        elif x in [10, 11]:
            return 1 if x == 10 else 0
        elif y in [10, 11]:
            return 0 if y == 10 else 1
        elif y in [12, 13]:
            return 1 if y == 12 else 0
        return 0

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

    def spawn_vehicle(self, veh_id, x, y):
        """Spawn a vehicle at the given grid position via TraCI."""
        X, Y = self.grid_to_sumo(x, y)
        edge_id = self.get_edge_id(x, y)
        lane_idx = self.get_lane_index(x, y)
        route_id = self.get_route_id(x, y)
        try:
            traci.vehicle.add(veh_id, route_id, typeID="car")
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
            
    def spawn_pedestrian(self, ped_id, x, y):
        """Spawn a pedestrian at the given grid position via TraCI.

        Since the network has no dedicated sidewalk infrastructure, pedestrians
        are placed on a vehicle edge with a long waiting stage, then teleported
        to their actual position with moveToXY(keepRoute=2).
        """
        try:
            # Place on a default edge at the midpoint — the initial position
            # does not matter because we immediately teleport with moveToXY.
            edge_id = "N_to_C"
            pos = 45.0  # roughly mid-edge (edge length ≈ 89.6 m)
            traci.person.add(ped_id, edge_id, pos, typeID="ped")
            # Person MUST have at least one plan stage or SUMO removes them.
            traci.person.appendWaitingStage(ped_id, 1000.0, "planner_controlled")
            # Teleport to the real grid position
            X, Y = self.grid_to_sumo(x, y)
            traci.person.moveToXY(ped_id, "", X, Y, angle=0, keepRoute=2)
        except traci.exceptions.TraCIException as e:
            print(f"  Warning: Could not spawn pedestrian {ped_id}: {e}")
        
    def move_pedestrian(self, ped_id, x, y):
        X, Y = self.grid_to_sumo(x, y)
        try:
            traci.person.moveToXY(ped_id, "", X, Y, angle=0, keepRoute=2)
        except traci.exceptions.TraCIException:
            pass

    def spawn_background_vehicles(self, density="medium", step=0, seed=None):
        """Dynamically spawns background traffic at boundary nodes based on the density setting."""
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
            try:
                traci.vehicle.add(veh_id, route_id, typeID="car")
            except traci.exceptions.TraCIException:
                pass
