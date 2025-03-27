# README.md


## Indoor Pathfinding Simulation


This project demonstrates pathfinding and steering behaviors in an indoor environment. It includes implementations of Dijkstra's algorithm and A* pathfinding with different heuristics.


### Prerequisites

- C++ compiler with C++17 support

- SFML library for graphics (for the visual simulation)

- Make build system


### Project Structure

- `q4.cpp` - Indoor pathfinding simulation with visual interface

- `dijkstra_avg.cpp` - Dijkstra's algorithm implementation with metrics

- `a_star.cpp` - A* pathfinding with landmark and minimum edge heuristics

### Building the Project

To build all executables:
```sh
make all
```

This will compile all the source files and create the following executables


### Running the Simulation


To run all programs in sequence with predefined inputs: I have already fed th einputs in the command line with the example nodes i have pasted as screenshots in my write up just sit back and enjoy!!

```sh
make run
```

This will execute:

1. `dijkstra_avg` with airline_routes.txt (source: 1, goal: 17)

2. `dijkstra_avg` with random_graph.txt (source: 0, goal: 13626)

3. `a_star` with airline_routes.txt (source: 1, goal: 17)

4. `a_star` with random_graph.txt (source: 0, goal: 13626)

5. `q4` - The visual indoor pathfinding simulation

### Using the Visual Simulation (q4)

When running the `q4` program:


1. A window will open showing an indoor environment with rooms and corridors

2. Click anywhere in the environment to set a target position

3. The boid will find a path to the target and follow it

4. Blue dots show the character's trail (breadcrumbs)

5. Green dots show the calculated path

### Cleaning Up

To remove all compiled files:
```sh
make clean
```


This will delete all object files and executables.


### Algorithm Comparison

The project demonstrates different pathfinding algorithms:

- Dijkstra's algorithm (in `dijkstra_avg`)

- A* with landmark-based heuristic (in `a_star`)

- A* with minimum outgoing edge heuristic (in `a_star`)

Each algorithm outputs metrics including:

- Runtime

- Nodes processed

- Maximum fringe size

- Fill count (unique nodes visited)

- Focus ratio (for A*) = number of nodes on path/ total number of nodes explored

These metrics help compare the efficiency of different pathfinding approaches.