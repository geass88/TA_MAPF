# MAPF Benchmark Instances

This repository accompanies the paper:

**Solving the Multi-Agent Pathfinding Problem with Time-Expanded Networks**
*Tommaso Adamo, Roberto Baldacci, Gianpaolo Ghiani, Emanuela Guerriero*

* Dipartimento di Ingegneria dell’Innovazione, Università del Salento, Lecce, Italy
* Division of Engineering Management and Decision Sciences, Hamad Bin Khalifa University, Doha, Qatar


## Sources
This repository provides benchmark instances for the **Multi-Agent Path Finding (MAPF)** problem.
The instances are expressed in **YAML format**.
In detail, we tested two types of instances taken from the MAPF literature:

* **Grid environments**, introduced by [*Hönig et al. (2018)*](https://dl.acm.org/doi/10.5555/3237383.3237495), see also: [https://github.com/atb033/multi_agent_path_planning/tree/master/centralized/benchmark](https://github.com/atb033/multi_agent_path_planning/tree/master/centralized/benchmark)
* **Warehouse environments**, introduced by [*Stern et al. (2019)*](https://doi.org/10.1609/socs.v10i1.18510), see also: [https://movingai.com/benchmarks/mapf/index.html](https://movingai.com/benchmarks/mapf/index.html)

These benchmark families are widely used in the MAPF community to evaluate and compare algorithms.

## Instance Format

Each instance specifies:

* **agents**:

  * `name`: agent identifier
  * `start`: starting coordinates `[x, y]`
  * `goal`: target coordinates `[x, y]`

* **map**:

  * `dimensions`: grid size `[width, height]`
  * `obstacles`: list of blocked cells `[x, y]`

**Note:** all coordinates (`start`, `goal`, `obstacles`) use **zero-based indexing**.

### Example

```yaml
agents:
-   goal: [7, 6]
    name: agent0
    start: [2, 6]
map:
    dimensions: [8, 8]
    obstacles:
    - [5, 3]
    - [4, 3]
    - [7, 1]
    - [4, 4]
    - [3, 7]
    - [5, 4]
    - [4, 6]
    - [7, 3]
    - [1, 2]
    - [1, 7]
    - [2, 1]
    - [3, 2]
```

In this example, the map is an 8×8 grid with 12 obstacles and a single agent that must move from `[2, 6]` to `[7, 6]`.
