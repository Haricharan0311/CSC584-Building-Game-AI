#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <SFML/System/Vector2.hpp>

// Struct for Steering behavior
struct Steering {
    sf::Vector2f linear; // Linear velocity
    float angular;       // Angular velocity
};

// Struct for Kinematic properties
struct Kinematic {
    sf::Vector2f pos;       // Position
    sf::Vector2f vel;       // Velocity
    float orient;           // Orientation (in radians)
    float rotation;         // Angular rotation

    // Update function for kinematics
    void update(Steering& steering, float dtime);
};

#endif // KINEMATICS_H
