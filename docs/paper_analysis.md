# Research Paper Analysis: Multi-Agent Teamwise Cooperative Path Finding and Traffic Intersection Coordination

**Authors**: Zhongqiang Ren, Yilin Cai, Hesheng Wang  
**Venue**: IROS 2024, Abu Dhabi, UAE  

---

## 1. Problem Statement
The paper defines **Multi-Agent Teamwise Cooperative Path Finding (TCPF)**, which is a generalization of Multi-Agent Path Finding (MAPF). 
- A set of $N$ agents $I = \{1, 2, \dots, N\}$ operate on a workspace graph $G = (V, E)$.
- Agents are divided into $M$ teams $\{T_1, T_2, \dots, T_M\}$, where each team $T_j \subseteq I$ includes a subset of agents. Teams are not required to be mutually disjoint, and agents can belong to multiple teams.
- Each team has its own objective function $g^{T_j}$ (e.g., min-sum or min-max of individual path costs of its members).
- Since there are multiple teams with different objectives, TCPF is a multi-objective optimization problem. It seeks to find a **Pareto-optimal front** (or a set of cost-unique Pareto-optimal solutions) that represents the trade-offs among teams.

---

## 2. Motivation
Autonomous vehicle coordination at signal-free/unsignalized intersections:
- Vehicles from different directions naturally form separate teams.
- Each team seeks to minimize its own traversal time through the intersection, without concerning itself with the traversal times of other teams.
- Agent-agent collisions must be avoided, which requires trading off the traversal time of one team for another.
- Centralized scheduling can reveal the Pareto-optimal front, exposing the trade-offs (e.g., prioritized crossings, cut-ins, or detours) for decision-making.

---

## 3. TCPF Framework
- **Dominance**: A solution $\pi$ (joint path) dominates another solution $\pi'$ if its cost vector $\vec{g}(\pi)$ is component-wise less than or equal to $\vec{g}(\pi')$ and strictly less in at least one component.
- **Pareto-optimal set** $\Pi_*$: The set of all non-dominated conflict-free solutions.
- **Pareto-optimal front** $C^*$: The set of objective vectors corresponding to $\Pi_*$.

---

## 4. Algorithms

### A. Teamwise Cooperative CBS (TC-CBS)
TC-CBS is a two-level search framework similar to standard Conflict-Based Search (CBS) but with key differences:
1. **Cost Vector**: Instead of a scalar cost, each search node stores a team-based cost vector $\vec{g}$.
2. **Lexicographical Open List**: The high-level search node queue (`OPEN`) is sorted in lexicographical order of the cost vector $\vec{g}$.
3. **Filtering**: Search nodes that are dominated by or equal to already found Pareto-optimal solutions in a set $C$ are pruned/filtered.
4. **Termination**: TC-CBS terminates when `OPEN` is empty, returning all found Pareto-optimal solutions.

### B. Incompleteness of TC-CBS
For TCPF problems that are **not fully cooperative** (i.e., at least one team does not contain all agents), TC-CBS can fail to terminate in finite time even if the instance is solvable. This is because there can be infinitely many joint paths whose objective vectors are non-dominated by the Pareto-optimal front, leading to an infinite expansion of search nodes (e.g., when one agent's destination block forces another agent to wait indefinitely, generating infinite wait cycles).

### C. TC-CBS-t Algorithm (The Solution)
To resolve incompleteness, the paper proposes **TC-CBS-t**, which applies a transformation to the team objective functions:
$$g_f(\pi^{T_j}) := g(\pi^{T_j}) + \epsilon \sum_{i \notin T_j} g(\pi^i)$$
where:
- $\epsilon > 0$ is a small positive weight.
- $\sum_{i \notin T_j} g(\pi^i)$ is the sum of path costs of agents not in team $T_j$.

**Properties of TC-CBS-t**:
- **Completeness**: Because every team's transformed objective includes all agents, the problem becomes "fully cooperative". TC-CBS-t is guaranteed to terminate in finite time.
- **Pareto-optimality**: All untransformed vectors returned are guaranteed to be part of the original Pareto-optimal front $C^*$.
- **Subsetting**: TC-CBS-t may only find a subset of the Pareto-optimal front. However, if $\epsilon$ is chosen such that $M_{C^*} N \epsilon < m_{C^*}$ (where $M_{C^*}$ is the maximum path cost and $m_{C^*}$ is the minimum non-zero cost difference), TC-CBS-t is guaranteed to find the *entire* Pareto-optimal front.

---

## 5. Experimental Setup & Results
- **Simulators**: CARLA simulator using a 20-by-24 grid-map representation of a bidirectional cross intersection (Town11 map).
- **Settings**: Evaluated with up to 40 agents. Tested scenarios included:
  1. *Motorcade vs. Public*: Team 1 (motorcade, agents 1-3) and Team 2 (public vehicles, agents 4-7).
  2. *Bi-objective MAPF*: Two teams, one optimizing min-sum and the other optimizing min-max.
  3. *Each Agent as a Team*: $N$ teams for $N$ agents.
- **Results**: Showed that larger $\epsilon$ values speed up the search by pruning more search tree branches, but might result in finding fewer Pareto-optimal solutions.

---

## 6. Limitations of the Base Paper
1. **Lack of Pedestrian Coordination**: The environment is vehicle-only. Real-world urban intersections feature active pedestrian crossings that impact vehicular trajectories.
2. **Absence of Safety Metrics**: The optimization is purely based on travel time/path cost. It does not measure physical safety parameters like Time-to-Collision (TTC) or Post-Encroachment Time (PET).
3. **High-Level Search Scalability (TC-CBS)**: Traditional CBS branching can explode in high-density conflicts. The paper does not utilize modern CBS improvements like Conflict-Based Search with conflict prioritization (ICBS) to resolve conflicts more efficiently.
