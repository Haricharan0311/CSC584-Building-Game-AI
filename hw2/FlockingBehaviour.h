#ifndef FLOCKING_BEHAVIOUR_H
#define FLOCKING_BEHAVIOUR_H
#include "Kinematic.h" 
#include "SteeringBehaviour.h"
#include <vector>

// Utility function to maintain consistent speed
sf::Vector2f maintainSpeed(sf::Vector2f velocity, float targetSpeed) {
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > 0) {
        velocity = (velocity / speed) * targetSpeed;
    }
    return velocity;
}


class SeparationBehaviour : public SteeringBehaviour {
    public:
        void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
            linear = sf::Vector2f(0.0f, 0.0f);
        }
    
        void apply(Kinematic& kinematic, const std::vector<Kinematic>& neighbors) {
            sf::Vector2f steering(0.0f, 0.0f);
            const float minSeparation = 25.0f;
            const float maxRepelForce = maxAcceleration * 2.0f;
    
            for (const auto& neighbor : neighbors) {
                sf::Vector2f diff = kinematic.pos - neighbor.pos;
                float distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                if (distance > 0 && distance < minSeparation) {
                    diff /= distance;  // Normalize direction
                    float strength = std::min(maxRepelForce, maxRepelForce / (distance * distance));
                    steering += diff * strength;
                }
            }
    
            // Damping for smooth separation
            steering *= 0.6f;
    
            // Clamp the separation force
            float length = std::sqrt(steering.x * steering.x + steering.y * steering.y);
            if (length > maxAcceleration) {
                steering = (steering / length) * maxAcceleration;
            }
    
            linear = steering;
        }
    };

class AlignmentBehaviour : public SteeringBehaviour {
    public:
        void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
            linear = sf::Vector2f(0.0f, 0.0f);
        }
    
        void apply(Kinematic& kinematic, const std::vector<Kinematic>& neighbors) {
            sf::Vector2f avgVelocity(0.0f, 0.0f);
            int count = 0;
            for (const auto& neighbor : neighbors) {
                float distance = std::sqrt(
                    std::pow(kinematic.pos.x - neighbor.pos.x, 2) + 
                    std::pow(kinematic.pos.y - neighbor.pos.y, 2)
                );
                if (distance < radiusOfDeceleration) {
                    avgVelocity += neighbor.vel;
                    count++;
                }
            }
            if (count > 0) {
                avgVelocity /= static_cast<float>(count);
                avgVelocity = maintainSpeed(avgVelocity, SteeringBehaviour::maxVelocity);
    
                // Velocity matching
                sf::Vector2f steering = (avgVelocity - kinematic.vel) * 0.1f;
    
                // Clamp alignment force
                float length = std::sqrt(steering.x * steering.x + steering.y * steering.y);
                if (length > maxAcceleration * 0.6f) {
                    steering = (steering / length) * (maxAcceleration * 0.6f);
                }
                linear = steering;
            }
        }
    };
        
class CohesionBehaviour : public SteeringBehaviour {
    public:
        void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
            linear = sf::Vector2f(0.0f, 0.0f);
        }
    
        void apply(Kinematic& kinematic, const std::vector<Kinematic>& neighbors) {
            sf::Vector2f centerOfMass(0.0f, 0.0f);
            int count = 0;
            for (const auto& neighbor : neighbors) {
                float distance = std::sqrt(
                    std::pow(kinematic.pos.x - neighbor.pos.x, 2) + 
                    std::pow(kinematic.pos.y - neighbor.pos.y, 2)
                );
                if (distance < radiusOfDeceleration * 0.7f) {
                    centerOfMass += neighbor.pos;
                    count++;
                }
            }
            if (count > 0) {
                centerOfMass /= static_cast<float>(count);
                sf::Vector2f direction = centerOfMass - kinematic.pos;
                direction = maintainSpeed(direction, SteeringBehaviour::maxVelocity);
    
                // Cohesion steering
                sf::Vector2f steering = (direction - kinematic.vel) * 0.05f;
    
                // Clamp cohesion force
                float length = std::sqrt(steering.x * steering.x + steering.y * steering.y);
                if (length > maxAcceleration * 0.5f) {
                    steering = (steering / length) * (maxAcceleration * 0.5f);
                }
                linear = steering;
            }
        }
    };

class FlockingBehaviour : public SteeringBehaviour {
private:
    SeparationBehaviour separation;
    AlignmentBehaviour alignment;
    CohesionBehaviour cohesion;

    float separationWeight = 2.8f;  // Prioritize separation
    float alignmentWeight = 1.4f;
    float cohesionWeight = 2.0f;  

    std::vector<Kinematic> neighbors;

public:
    void setNeighbors(const std::vector<Kinematic>& newNeighbors) {
        neighbors = newNeighbors;
    }

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        sf::Vector2f sepForce, alignForce, cohForce;

        separation.apply(kinematic, neighbors);
        sepForce = separation.linear;

        alignment.apply(kinematic, neighbors);
        alignForce = alignment.linear;

        cohesion.apply(kinematic, neighbors);
        cohForce = cohesion.linear;

        linear = (separationWeight * sepForce) + (alignmentWeight * alignForce) + (cohesionWeight * cohForce);

        if (std::sqrt(linear.x * linear.x + linear.y * linear.y) > maxAcceleration) {
            float length = std::sqrt(linear.x * linear.x + linear.y * linear.y);
            linear = (linear / length) * maxAcceleration;
        }
    }
};

#endif // FLOCKING_BEHAVIOUR_H
