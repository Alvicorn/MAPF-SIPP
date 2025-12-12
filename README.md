# MAPF-SIPP


## Getting Started
This repo is designed to run in a [VS Code Devcontainer](https://code.visualstudio.com/docs/devcontainers/containers).

The following steps are for initializing the development environment:
1. Pull the latest Docker image
    ```bash
    docker pull alvicorn/mapf-sipp:latest
    ```
2. Open the repo in VS Code and _rebuild as devcontainer_ if necessary. (VS Code normally detects the `.devcontainer/` and will open it according)

## Grid Map
The map used for this MAPF instance is statically provided as a text file containing the number of row, number of columns and then the map itself as ascii characters. `.` denotes a free space whereas `@` denotes a static obstacle. For example:
```txt
8 8
. . @ . . . . .
. . . @ . . . .
. . @ . . . . .
. . . . . . . .
. . . . . . . @
. . . . . @ . .
. . . . . . @ .
. . . . . . . .

```


## Generating Dynamic Environments
Dynamic environments are statically provided to the MAPF solver via a TOML file. Using a static map as a base, the TOML file describes the agents and dynamic obstacles. Below is an example of a dynamic environment file:
```toml
[[agents]]
id = 0
start_point = {x = 2, y = 1}
goal_point = {x = 7, y = 3}


[[agents]]
id = 1
start_point = {x = 6, y = 2}
goal_point = {x = 2, y = 1}


[[dynamic_obstacles]]
id = "a"
start_point = {x = 1, y = 1}

  [[dynamic_obstacles.trajectories]]
  points = [
    {x = 5, y = 1, t = 5, p = 0.5},
    {x = 5, y = 3, t = 10, p = 0.1},
  ]

  [[dynamic_obstacles.trajectories]]
  points = [
    {x = 4, y = 1, t = 5, p = 0.5},
    {x = 4, y = 3, t = 10, p = 0.1},
  ]


[[dynamic_obstacles]]
id = "b"
start_point = {x = 2, y = 5}

  [[dynamic_obstacles.trajectories]]
  points = [
    {x = 3, y = 5, t = 3, p = 0.7},
    {x = 5, y = 3, t = 15, p = 0.2},
  ]

  [[dynamic_obstacles.trajectories]]
  points = [
    {x = 2, y = 5, t = 3, p = 0.6},
    {x = 4, y = 3, t = 15, p = 0.9},
  ]
```

Dynamic environment files can be written manually or randomly generated using `dynamic_env_generator.py`. See the following command line example:

```bash
python dynamic_env_generator.py --map instances/maps/custom_instances/simple-8-8.txt --out dynamic_envs/  --num_agents 5 --num_obstacles 27 --max_num_trajectories 2 --num_instances 100
```
`--map`: Path to the static MAPF map. This flag is mandatory.
<br>
`--out`: Output directory to dump the dynamic environment file instances. This flag is mandatory.
<br>
`--num_agents`: Number of agents to be included in each instance. Defaults to 1 if the flag is not provided.
<br>
`--num_obstacles`: Number of dynamic obstacles to be included in each instance. Defaults to 1 if the flag is not provided.
<br>
`--max_num_trajectories`: The maximum number of trajectories a dynamic obstacle may have. A random number of trajectories will be generated between [1, max_num_trajectories]. Defaults to 1 if the flag is not provided.
<br>
`--num_instances`: Number of dynamic environment instances to be included in each instance. Defaults to 1 if the flag is not provided.

## Running Experiments
Experiments of MAPF-SIPP can be run using the `run_dynamic_environment.py` script. For example:
```bash
python run_dynamic_experiments.py --map instances/maps/custom_instances/simple-8-8.txt --instance dynamic_envs/simple-8-8-1.toml
```
`--map`: Path to the static MAPF map. This flag is mandatory.
<br>
`--instance`: Path to the file containing the agent data and dynamic obstacles (TOML file). This flag is mandatory.

More work to follow to implement the missing MAPF-SIPP portion.
