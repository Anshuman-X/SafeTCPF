# SafeTCPF: Safety-Aware Teamwise Cooperative Path Finding

SafeTCPF is a safety-aware multi-agent path finding (MAPF) and traffic intersection coordination framework. It extends the state-of-the-art **Multi-Agent Teamwise Cooperative Path Finding (TCPF) system presented at IROS 2024** by incorporating dynamic pedestrian coordination, high-fidelity safety calculations, and search efficiency optimizations.

This repository implements **Teamwise Improved Conflict-Based Search (TC-ICBS)**, which features dynamic conflict classification, team-aware conflict prioritization, and bypass mechanics to solve teamwise path planning with dynamic obstacles under high congestion.

---

## 🚀 Key Enhancements & Contributions

### 1. Novel TC-ICBS Algorithm (Objectives 1 & 2)
* **Dynamic Early-Exit Classification**: Analyzes conflicts sequentially. The moment a *cardinal* conflict is identified, the planner branches immediately, saving up to 28 low-level Space-Time A* search calls per node.
* **Team-Aware Conflict Prioritization**: Dynamically orders conflicts, prioritizing *inter-team* conflicts (global coordination bottlenecks that shape the Pareto-optimal frontier) over *intra-team* conflicts (which can be resolved internally at lower cost).
* **Space-Time Path Caching**: Memoizes Space-Time A* search paths using a shared constraints hash map, avoiding redundant path planning.
* **Pruning and Bypassing**: Reuses precomputed classification paths during branching to prune dead-end nodes instantly and bypass branching altogether for non-cardinal conflicts.

### 2. Realistic SUMO Simulation (Objective 3)
* **Dynamic Background Traffic**: Spawns non-planning vehicles dynamically at boundary nodes using TraCI. These vehicles drive naturally through the intersection using SUMO's internal car-following and lane-changing models, providing a realistic traffic environment.
* **Poisson Spawning Distributions**: Implements configurable Poisson and binomial vehicle arrival rates.
* **Multi-Seed Configuration**: Fully supports reproducible, seed-based randomized traffic and pedestrian flows configured entirely via `config/config.yaml`.
* **Clean Spawning**: Eliminates hybrid teleportation warnings by synchronizing the route XML file headers and spawning entities cleanly via TraCI.

### 3. Fine-Grained Safety Metrics (Objective 4)
* **Decoupled V-V & V-P Calculations**: Decouples safety metrics into independent **Vehicle-Vehicle (V-V)** and **Vehicle-Pedestrian (V-P)** categories.
* **Quadratic TTC Solver**: Replaces simple vector projections with a mathematically correct quadratic overlap solver, treating vehicles as circles of radius $R_{veh} = 0.75$ (safety distance $d_{vv} = 1.5$) and pedestrians as circles of radius $R_{ped} = 0.25$ (safety distance $d_{vp} = 1.0$).
* **Comprehensive Safety Suite**: Tracks Near-Miss Counts, Collision Counts, Minimum/Average TTC & PET, Critical TTC/PET Events (TTC $\le 1.5$s, PET $\le 2.0$s), Queue Lengths, Average Delays, and Intersection Throughput.

### 4. Multi-Seed Statistical Runner (Objective 5, 8 & 10)
* **Robust Grid Search**: Evaluates scenarios across multiple random seeds, traffic densities, and pedestrian densities.
* **Rigorous Mathematical Validation**: Computes the Mean, Median, Standard Deviation, Minimum, Maximum, and 95% Confidence Intervals (using Student's t-distribution) for all 13 metrics.
* **NaN-Handling for Failed Runs**: Correctly represents failed planning runs (timeouts/max nodes) as `NaN` to prevent skewing average results, while reporting the `Success Rate` independently.

---

## 📂 Project Directory Structure

```
├── algorithms/              # Core Path-Finding Planners
│   ├── a_star.py            # Low-level SpaceTimeAStar with Path Caching
│   ├── search_node.py       # High-level CTNode with Team-Aware Heuristics
│   ├── tc_cbs.py            # TC-CBS-t Baseline Planner (IROS 2024)
│   └── tc_icbs.py           # Proposed TC-ICBS Planner (Optimized)
├── config/
│   └── config.yaml          # Central YAML Configuration File (Seeds, Densities, Flow)
├── evaluation/
│   └── experiment_runner.py # Multi-seed Statistical Runner & PDF Compiler
├── metrics/
│   └── safety_metrics.py    # Fine-grained Safety Metrics & Quadratic TTC Solver
├── pedestrians/
│   └── ped_model.py         # Seed-based Randomized Pedestrian Path Generator
├── simulation/
│   └── sumo_env.py          # SUMO Environment & TraCI background spawning
├── docs/
│   ├── diagrams.md          # 6 Mermaid System Architecture & Flow Diagrams
│   └── improvement_analysis.md # Research scientist strengths & weaknesses analysis
├── results/                 # Auto-generated 12 Statistical CSV Files
├── visualizations/          # Auto-generated 15 Premium Plot Figures (PNG)
├── reports/                 # Auto-generated LaTeX-style PDF Reports
├── tests/                   # Unit and Integration Test suite
├── main.py                  # Master Execution Pipeline Entry Point
└── requirements.txt         # Project Python Dependencies
```

---

## 🛠️ Installation & Setup

### Prerequisites
1. **Python 3.8+**
2. **Eclipse SUMO**: Install SUMO and ensure the `SUMO_HOME` environment variable is set.
   * On Windows: Add `C:\Program Files (x86)\Eclipse\Sumo\bin` (or your installation path) to your system `PATH`.

### Setup
1. Clone the repository and navigate into the project directory:
   ```bash
   cd SafeTCPF
   ```
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## 🏃 Running the Project

### 1. Execute the Entire Pipeline
To run the multi-seed experiments, generate all CSV files, visual charts, compile PDF reports, and launch the headless SUMO simulation demo, run:
```bash
python main.py
```

### 2. Run with SUMO GUI Demo
To watch the simulation demo in the SUMO-GUI window (showing planning vehicles coordinating with crossing pedestrians and dynamic background traffic):
```bash
python main.py --gui
```

---

## 📊 Output Artifacts & Validation

Once the pipeline completes, the following files will be automatically updated:

### 1. Statistical CSV Files (`results/`)
* **[summary.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/summary.csv)**: Overall aggregated summary of all major performance and safety metrics.
* **[travel_time.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/travel_time.csv)**: Aggregated travel times (Mean, Median, Std Dev, Min, Max, 95% CI) per density.
* **[waiting_time.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/waiting_time.csv)**: Aggregated vehicle waiting times.
* **[delay.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/delay.csv)**: Aggregated average delay.
* **[runtime.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/runtime.csv)**: CPU runtime and expanded nodes.
* **[memory.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/memory.csv)**: High-level search memory usage (MB).
* **[conflict_count.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/conflict_count.csv)**: Near-misses, conflicts, and collisions.
* **[vehicle_vehicle_ttc.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/vehicle_vehicle_ttc.csv)** & **[vehicle_pedestrian_ttc.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/vehicle_pedestrian_ttc.csv)**: Fine-grained TTC metrics.
* **[vehicle_vehicle_pet.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/vehicle_vehicle_pet.csv)** & **[vehicle_pedestrian_pet.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/vehicle_pedestrian_pet.csv)**: Fine-grained PET metrics.
* **[throughput.csv](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/results/throughput.csv)**: Intersection vehicle throughput capacity.

### 2. Publication-Quality Figures (`visualizations/`)
* Bar charts: [travel_time.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/travel_time.png), [waiting_time.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/waiting_time.png), [delay.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/delay.png), [average_speed.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/average_speed.png), [throughput.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/throughput.png), [queue_length.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/queue_length.png).
* Line plots (Complexity Trends): [runtime.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/runtime.png), [search_nodes.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/search_nodes.png), [memory_usage.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/memory_usage.png).
* Box plots (Distributions): [travel_time_dist.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/travel_time_dist.png), [runtime_dist.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/runtime_dist.png).
* Histograms (Safety spreads): [vehicle_vehicle_ttc.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/vehicle_vehicle_ttc.png), [vehicle_pedestrian_ttc.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/vehicle_pedestrian_ttc.png), [vehicle_vehicle_pet.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/vehicle_vehicle_pet.png), [vehicle_pedestrian_pet.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/vehicle_pedestrian_pet.png).
* Scatter plot (Safety-Efficiency Trade-off): [safety_efficiency_tradeoff.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/safety_efficiency_tradeoff.png).
* Success Rate: [success_rate.png](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/visualizations/success_rate.png).

### 3. Compiled PDF Reports (`reports/`)
* **[final_report.pdf](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/reports/final_report.pdf)**: Comprehensive research report containing abstract, methodology, results tables, visual plots, and discussion.
* **[base_paper_comparison.pdf](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/reports/base_paper_comparison.pdf)**: Head-to-head comparative analysis against the IROS 2024 base paper, providing mathematical justification and verified percentage improvements.

---

## 📈 Diagrams & Architecture Reference
For visual guides on the system structure, algorithm flow, class relationships, and execution sequences, please refer to the **[docs/diagrams.md](file:///d:/RESEARCH%20INTERNSHIP%20NITT/SafeTCPF/docs/diagrams.md)** documentation.
