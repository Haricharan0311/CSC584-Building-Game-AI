#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <SFML/System/Vector2.hpp>

class SteeringBehaviour; // Forward declaration

struct Kinematic {
    sf::Vector2f pos;
    sf::Vector2f vel;
    float orient;
    float rotation;

    //void update(SteeringBehaviour& steering, float dtime);
};

#endif // KINEMATICS_H