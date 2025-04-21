# README.md

## Monster AI Comparison – Behavior Tree vs Decision Tree

This project demonstrates three AI-driven monsters in an indoor environment:

- A Decision tree based boid (`decision`)
- A hand-coded Behavior Tree monster (`q4`)
- A learned Decision Tree monster using game-state data (`q5`)

The environment supports walls, rooms, pathfinding, steering behaviors, and visual debugging.

---

### Prerequisites

- C++17 compatible compiler  
- [SFML](https://www.sfml-dev.org/) library for rendering  
- Make build system  

---

### Project Structure

- `q4.cpp` - Runs the monster controlled by a Behavior Tree
- `q5.cpp` - Runs the monster controlled by a trained Decision Tree
- `Monster.*`, `DecisionTreeMonster.*` - AI and movement logic
- `BehaviorTree.*` - Implementation of the BT nodes (Patrol, Chase, etc.)
- `train_dt.cpp` - Loads CSV and builds the decision tree for predictions

---

### Building

To build both simulation binaries:
```sh
make all
```

---

### Running the Simulations
A fair warnimg, When i run `q5` the decision tree is trained on `monster_behavior.csv` and thus takes 30 seconds to launch the window. To maintain the flow professor had asked, the file `q4` writes into a csv file named `monster_behavior_dummy.csv`. This was done in the interest of time and the file `monster_behavior.csv` was pre run to collect a lot of data.
To run everything at once:
```sh
make run
```

To run the behavior tree monster:
```sh
./q4
```
To rrun the decision tree learner:
```sh
./train_dt
```

To run the decision tree monster:
```sh
./q5
```

In both, you can:
- Watch the player-controlled boid move
- Observe monster reactions (patrolling, chasing, dancing)
- See pathfinding and wall avoidance in action

---

### Controls


- The monster will respond based on its logic tree
- Breadcrumbs and debug text show internal state

---

### Cleaning Up

To remove all builds:
```sh
make clean
```

---

### Training Data

The decision tree monster uses `monster_behavior.csv` (generated from `q4`) to learn behaviors using `DecisionTreeLearner`.

---
