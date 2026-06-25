# SafeTCPF System Architecture & Diagram Documentation

This document contains high-fidelity Mermaid diagrams illustrating the architecture, algorithm flow, class relationships, sequence of execution, pipeline, and experimental workflow of the improved **SafeTCPF** framework.

---

## 1. System Architecture Diagram

This diagram shows the modular structure of the SafeTCPF framework and the data flow between different components.

```mermaid
graph TD
    A[main.py: Entry Point] --> B[evaluation/experiment_runner.py]
    A --> C[simulation/sumo_env.py]
    B --> D[algorithms/tc_icbs.py: TC-ICBS Planner]
    B --> E[algorithms/tc_cbs.py: TC-CBS-t Baseline]
    B --> F[pedestrians/ped_model.py: Pedestrian Model]
    B --> G[metrics/safety_metrics.py: Safety Evaluator]
    
    D --> H[algorithms/a_star.py: SpaceTimeAStar]
    E --> H
    
    C --> I[TraCI API Controller]
    I --> J[SUMO GUI / SUMO Headless]
    
    style A fill:#2c3e50,stroke:#34495e,stroke-width:2px,color:#fff
    style B fill:#3498db,stroke:#2980b9,stroke-width:2px,color:#fff
    style C fill:#e67e22,stroke:#d35400,stroke-width:2px,color:#fff
    style D fill:#2ecc71,stroke:#27ae60,stroke-width:2px,color:#fff
```

---

## 2. Algorithm Flow Diagram (TC-ICBS)

This diagram details the step-by-step logic of the proposed Teamwise Improved Conflict-Based Search (TC-ICBS) planner, highlighting our **early-exit classification** and **dynamic conflict ordering** optimizations.

```mermaid
flowchart TD
    A([Start Planner]) --> B[Initialize Root CTNode]
    B --> C[Find Individual Space-Time A* Paths]
    C --> D[Compute Transformed Team Costs & Conflicts]
    D --> E[Push Root to OpenList Heap]
    
    E --> F{OpenList Empty or<br/>Timeout/MaxNodes?}
    F -- Yes --> G([Return Pareto-Optimal Set C])
    F -- No --> H[Pop Node with Lowest Transformed Cost]
    H --> I{Dominance Check<br/>against C?}
    I -- Yes --> F
    I -- No --> J[Find All Paths Conflicts]
    J --> K{Are there any<br/>Conflicts?}
    
    K -- No --> L[Solution Found! Update C]
    L --> F
    
    K -- Yes --> M[Dynamic Conflict Ordering:<br/>Prioritize Inter-team over Intra-team]
    M --> N[Sequential Conflict Classification<br/>with Early-Exit]
    N --> O{Found Cardinal<br/>Conflict?}
    O -- Yes --> P[Branch into 2 Children]
    O -- No --> Q{Found Non-Cardinal<br/>Conflict for Bypass?}
    
    Q -- Yes --> R[Try Bypass: Update paths in-place]
    R --> S{Bypass Successful?}
    S -- Yes --> T[Push Updated Node to OpenList]
    S -- No --> U[Branch on first Semi-cardinal/Non-cardinal]
    Q -- No --> U
    
    P --> V[For each child, re-plan constrained agent]
    U --> V
    V --> W{New path found?}
    W -- Yes --> X[Compute costs & check dominance]
    X --> Y[Push Child to OpenList]
    W -- No --> Z[Prune Branch]
    
    T --> F
    Y --> F
    Z --> F
```

---

## 3. Class Diagram

This UML class diagram shows the relationships and major attributes/methods of the core object-oriented modules.

```mermaid
classDiagram
    class CTNode {
        +dict paths
        +dict constraints
        +list cost_vector
        +list transformed_cost_vector
        +int num_conflicts
        +int num_inter_team_conflicts
        +copy() CTNode
        +__lt__(other) bool
    }
    
    class SpaceTimeAStar {
        +int grid_width
        +int grid_height
        +dict cache
        +is_on_road(x, y) bool
        +get_neighbors(x, y) list
        +heuristic(x, y, goal) int
        +find_path(start, goal, v_constrs, e_constrs, dyn_obstacles) list
    }
    
    class TCICBSPlanner {
        +list teams
        +list agents_def
        +float epsilon
        +SpaceTimeAStar low_level
        +get_agent_team(agent_id) int
        +is_inter_team_conflict(conflict) bool
        +count_inter_team_conflicts(paths) int
        +classify_conflict(node, conflict, dyn_obstacles) tuple
        +select_best_conflict(node, conflicts, dyn_obstacles) tuple
        +plan(max_nodes, time_limit, dyn_obstacles) list
    }
    
    class SumoEnvironment {
        +dict config
        +start_simulation(gui)
        +step()
        +stop_simulation()
        +spawn_vehicle(veh_id, x, y)
        +move_vehicle(veh_id, x, y, angle)
        +spawn_pedestrian(ped_id, x, y)
        +move_pedestrian(ped_id, x, y)
        +spawn_background_vehicles(density, step, seed)
    }
    
    class PedestrianModel {
        +int grid_width
        +int grid_height
        +generate_pedestrians(density, sim_steps, seed) list
        +get_occupancy_set(pedestrians) set
    }
    
    class ExperimentRunner {
        +dict config
        +run() DataFrame
        +generate_and_save_csv_outputs(raw_df)
        +generate_visualizations(raw_df)
        +generate_pdf_reports(raw_df)
    }
    
    TCICBSPlanner *-- SpaceTimeAStar : Uses
    TCICBSPlanner ..> CTNode : Manipulates
    ExperimentRunner --> TCICBSPlanner : Coordinates
    ExperimentRunner --> PedestrianModel : Generates obstacles
    ExperimentRunner --> SumoEnvironment : Demonstrates
```

---

## 4. Sequence Diagram (Master Pipeline)

This sequence diagram traces the execution of the entire execution sequence from the `main.py` entry point.

```mermaid
sequenceDiagram
    autonumber
    actor User
    participant Main as main.py
    participant Runner as ExperimentRunner
    participant Planner as TCICBSPlanner
    participant AStar as SpaceTimeAStar
    participant Safety as safety_metrics.py
    participant SUMO as SumoEnvironment
    
    User->>Main: python main.py --gui
    
    Note over Main, Runner: 1. Run Offline Experiments Grid Search
    Main->>Runner: run()
    loop For each Scenario, Algo, Traffic, Ped Density, and Seed
        Runner->>Planner: plan(dyn_obstacles)
        loop High-Level CTNode Expansion
            Planner->>AStar: find_path(constraints)
            AStar-->>Planner: path or None
        end
        Planner-->>Runner: solutions list
        Runner->>Safety: calculate_metrics(best_sol)
        Safety-->>Runner: metrics dict
    end
    Runner-->>Main: raw_results DataFrame
    
    Note over Main, Runner: 2. Export Statistics & Visualizations
    Main->>Runner: generate_visualizations(df)
    Runner-->>Main: Saved PNGs in visualizations/
    Main->>Runner: generate_pdf_reports(df)
    Runner-->>Main: Saved reports/final_report.pdf & comparison.pdf
    
    Note over Main, SUMO: 3. Run SUMO Demonstration
    Main->>SUMO: start_simulation(gui=True)
    Main->>SUMO: spawn_vehicle() & spawn_pedestrian()
    loop For t = 0 to max_t
        Main->>SUMO: move_vehicle() & move_pedestrian()
        Main->>SUMO: spawn_background_vehicles()
        SUMO->>SUMO: Simulation Step (yields & car-following)
    end
    Main->>SUMO: stop_simulation()
    Main-->>User: Prints "PROJECT IMPROVEMENT COMPLETE"
```

---

## 5. Pipeline Diagram & Experiment Workflow

This diagram maps the experimental grid search pipeline, showing how raw simulation data is processed, aggregated, and compiled into peer-reviewed figures and reports.

```mermaid
graph TD
    A[Scenarios: Motorcade, FourTeams] --> B[Grid Search Generator]
    C[Algorithms: TC-CBS-t, TC-ICBS] --> B
    D[Traffic Densities: Low, Med, High] --> B
    E[Pedestrian Densities: Low, Med, High] --> B
    F[Random Seeds: 42, 101, 202] --> B
    
    B --> G[Run Solver & Profile CPU/Memory]
    G --> H[Run Safety Metrics Separator V-V / V-P]
    H --> I[Store Raw Run Row in DataFrame]
    
    I --> J[Aggregation Loop: Group by Scenario, Algo, Densities]
    J --> K[Compute Mean, Median, Std Dev, Min, Max, 95% Confidence Intervals]
    
    K --> L[Export 11 Detailed CSVs + summary.csv]
    K --> M[Generate 15 Premium Visual Plots]
    
    L --> N[Compile reportlab PDF Reports]
    M --> N
    
    N --> O[reports/final_report.pdf]
    N --> P[reports/base_paper_comparison.pdf]
    
    style B fill:#f1c40f,stroke:#f39c12,stroke-width:2px
    style K fill:#9b59b6,stroke:#8e44ad,stroke-width:2px,color:#fff
    style N fill:#e74c3c,stroke:#c0392b,stroke-width:2px,color:#fff
    style O fill:#2ecc71,stroke:#27ae60,stroke-width:2px,color:#fff
    style P fill:#2ecc71,stroke:#27ae60,stroke-width:2px,color:#fff
```
