#include "Update.h"
#include <cmath>

void update(Kinematic& character, SteeringBehaviour& steering, float deltaTime) {
    character.vel += steering.linear * deltaTime;
    character.rotation += steering.angular * deltaTime;

    // Limit velocity to maxVelocity
    float speed = std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
    if (speed > SteeringBehaviour::maxVelocity) {
        character.vel = character.vel / speed * SteeringBehaviour::maxVelocity;
    }

    // Damping
    if (std::abs(steering.linear.x) < 0.1f && std::abs(steering.linear.y) < 0.1f) {
        character.vel *= 0.95f;
        if (speed < 1.0f) {
            character.vel = sf::Vector2f(0, 0);
        }
    }

    character.pos += character.vel * deltaTime;

    // Update orientation to match direction of velocity
    if (speed > 5.0f) {
        float targetOrient = std::atan2(character.vel.y, character.vel.x);
        character.orient = targetOrient;
    } else {
        character.rotation = 0;
    }
}
