#ifndef PATROL_POINT_H
#define PATROL_POINT_H

#include <SFML/System/Vector2.hpp>

struct PatrolPoint {
    sf::Vector2f position;
    float waitTime;
    
    PatrolPoint(const sf::Vector2f& pos, float wait = 2.0f) 
        : position(pos), waitTime(wait) {}
};

#endif // PATROL_POINT_H
