# Steering and Flocking Behaviors Simulation

This project demonstrates various steering and flocking behaviors using the SFML (Simple and Fast Multimedia Library). The simulations showcase different types of motion behaviors for game AI, including velocity matching, wandering, and boid flocking behaviors.

---

## Overview

This project contains the following executables:
- **q1**: Velocity Matching
- **q2**: Mouse Control with Breadcrumbs
- **q2_b**: Extended Mouse Control 
- **q3**: Random and Smooth Wandering
- **q4**: Flocking Simulation

---

## Descriptions

### q1 - Velocity Matching
This simulation demonstrates velocity matching behavior where an entity adjusts its velocity to match that of a target, typically used in leader-following scenarios.

### q2 - Mouse Control with Breadcrumbs
The entity follows the mouse click location, leaving a trail of breadcrumbs. It demonstrates the usage of position and orientation behaviors.

### q2_b - Extended Mouse Control
An extension of `q2` with a different set of free parameters, allowing the entity to navigate more smoothly towards mouse click locations.

### q3 - Random and Smooth Wandering
Shows the difference between random wandering and smooth wandering behaviors, allowing the entity to move in a more natural, lifelike manner.

### q4 - Flocking Simulation
A flocking simulation using boids algorithm, demonstrating separation, alignment, and cohesion behaviors for group movement.

---

## Requirements

- **SFML** (Simple and Fast Multimedia Library)
- C++17 compiler (e.g., `g++` or `clang++`)
- Make


## Compilation

Compile all executables using the Makefile:

```sh
make all
```

This will compile and link all the source files, creating the following executables:
- `q1`
- `q2`
- `q2_b`
- `q3`
- `q4`

---

### Individual Compilation

To compile a specific executable, use:
```sh
make q1      # For q1
make q2      # For q2
make q2_b    # For q2_b
make q3      # For q3
make q4      # For q4
```

---

## Running the Executables

After compilation, if you want to run the files individually you can run the executables as follows:

```sh
./q1
./q2
./q2_b
./q3
./q4
```

Alternatively, run all executables sequentially with:
```sh
make run
```

I'd prefer using this ! Makes life way easier.

---

## Clean Up

To remove all object files and executables:
```sh
make clean
```

---

## Controls and Interactions

### q1 - Velocity Matching
- The entity matches the velocity of the mouse.
- No user input required.

### q2 / q2_b - Mouse Control
- Click to set a target location.
- The entity will arrive at the clicked location, leaving breadcrumbs behind.

### q3 - Wandering
- Press `Space` to toggle between random and smooth wandering.

### q4 - Flocking Simulation
- Watch as the boids exhibit flocking behavior with separation, alignment, and cohesion. It usually takes a few seconds to start flocking so please do wait !! This was made intentionally as it seemed more natural for the boids to flock naturally and not to snap from their movement trajectory immediately or aggresively.

---

## Notes

- Make sure to have the texture file `boid-sm.png` in the same directory as the executables.
- Make sure to have the arial.ttf font loaded and doenloaded in the same directory as the code

---
