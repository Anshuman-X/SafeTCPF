#!/usr/bin/env python
"""
Standalone Configurable Traffic and Pedestrian Generator for SafeTCPF.
This script reads the configuration file config/config.yaml, generates
realistic config-driven mixed vehicles and continuous pedestrians, and
writes the route files net.rou.xml and net.ped.xml in the sumo_files directory.

Usage:
    python generate_traffic.py [options]
"""

import os
import sys
import yaml
import argparse

# Add parent directory to path to ensure modules are importable
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from simulation.sumo_env import SumoEnvironment

def main():
    parser = argparse.ArgumentParser(description="Configurable Traffic & Pedestrian Generator for SUMO Simulation")
    parser.add_argument("--config", type=str, default="config/config.yaml", help="Path to config file")
    parser.add_argument("--seed", type=int, help="Override random seed")
    parser.add_argument("--duration", type=int, help="Override duration in steps")
    parser.add_argument("--traffic-density", type=str, choices=["low", "medium", "high"], help="Override traffic density")
    parser.add_argument("--ped-density", type=str, choices=["low", "medium", "high"], help="Override pedestrian density")
    args = parser.parse_args()

    # Load configuration
    print(f"Loading configuration from {args.config}...")
    if not os.path.exists(args.config):
        print(f"Error: Configuration file {args.config} does not exist.")
        sys.exit(1)

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    # Apply overrides
    if 'traffic_generation' not in config:
        config['traffic_generation'] = {}

    if args.seed is not None:
        config['traffic_generation']['seed'] = args.seed
        print(f"  [Override] Seed: {args.seed}")
    if args.duration is not None:
        config['traffic_generation']['duration_steps'] = args.duration
        print(f"  [Override] Duration: {args.duration}")
    if args.traffic_density is not None:
        config['traffic_generation']['selected_traffic_density'] = args.traffic_density
        print(f"  [Override] Traffic Density: {args.traffic_density}")
    if args.ped_density is not None:
        config['traffic_generation']['selected_pedestrian_density'] = args.ped_density
        print(f"  [Override] Pedestrian Density: {args.ped_density}")

    # Write the modified temporary config if overridden, or use active config
    temp_config_path = "config/temp_config.yaml"
    with open(temp_config_path, "w") as f:
        yaml.safe_dump(config, f)

    try:
        # Instantiate SumoEnvironment, which internally calls generate_xml_files() and compile_network()
        print("Initializing SumoEnvironment and generating dynamic routes...")
        env = SumoEnvironment(config_path=temp_config_path)
        
        # Verify routes were written
        print("\nTraffic Generation Completed Successfully!")
        print(f"  - Vehicles Route File:   {env.rou_xml}")
        print(f"  - Pedestrians Route File: {env.ped_xml}")
        print(f"  - SUMO Config File:      {env.sumocfg}")
        
        # Print summary of what was generated
        gen_cfg = config['traffic_generation']
        seed = gen_cfg.get('seed', 42)
        duration = gen_cfg.get('duration_steps', 1000)
        t_density = gen_cfg.get('selected_traffic_density', 'medium')
        p_density = gen_cfg.get('selected_pedestrian_density', 'medium')
        
        print(f"\nParameters Used:")
        print(f"  - Seed: {seed}")
        print(f"  - Duration: {duration} steps")
        print(f"  - Traffic Density: {t_density} (spawn rate: {gen_cfg.get('vehicle_spawn_rates', {}).get(t_density)})")
        print(f"  - Pedestrian Density: {p_density} (spawn rate: {gen_cfg.get('pedestrian_spawn_rates', {}).get(p_density)})")
        
    finally:
        # Clean up temporary config file
        if os.path.exists(temp_config_path):
            os.remove(temp_config_path)

if __name__ == "__main__":
    main()
