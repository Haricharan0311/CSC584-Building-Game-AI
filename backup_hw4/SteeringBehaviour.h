#ifndef STEERING_BEHAVIOUR_H
#define STEERING_BEHAVIOUR_H

struct Kinematic;
#include <SFML/System/Vector2.hpp>
#include <cmath>
// static float timeToTargetVelocity = 0.2f;         
// static float timeToTargetRotation = 0.3f;          
// static float radiusOfSatisfaction = 15.0f;           
// static float radiusOfDeceleration = 100.0f; 

// static float timeToTargetVelocity = 0.05f;          
// static float timeToTargetRotation = 0.05f;         
// static float radiusOfSatisfaction = 5.0f;           
// static float radiusOfDeceleration = 50.0f; 
// Abstract base class for Steering Behavior
class SteeringBehaviour {
public:
    sf::Vector2f linear;  // Linear velocity
    float angular;        // Angular velocity

    static float maxVelocity;
    static float maxAcceleration;
    static float maxAngular;
    static float maxRotation;

    static float timeToTargetVelocity;         
    static float timeToTargetRotation;          
    static float radiusOfSatisfaction;           
    static float radiusOfDeceleration; 

    virtual void apply(Kinematic& kinematic, const Kinematic& target, float dtime) = 0;
    virtual ~SteeringBehaviour() = default;
};

// Subclass for Position Behavior
class PositionBehaviour : public SteeringBehaviour {
public:
    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        sf::Vector2f direction = target.pos - kinematic.pos;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        float goalSpeed;

        if (distance < radiusOfSatisfaction) {
            goalSpeed = 0;
        } else if (distance > radiusOfDeceleration) {
            goalSpeed = maxVelocity;
        } else {
            goalSpeed = maxVelocity * (distance / radiusOfDeceleration);
        }

        sf::Vector2f goalVelocity = direction;
        float length = std::sqrt(goalVelocity.x * goalVelocity.x + goalVelocity.y * goalVelocity.y);
        if (length > 0) {
            goalVelocity /= length;
        }
        goalVelocity *= goalSpeed;

        linear = goalVelocity - kinematic.vel;
        linear /= timeToTargetVelocity;
    }
};

// Subclass for Orientation Behavior
class OrientationBehaviour : public SteeringBehaviour {
public:
    float mapToRange(float rotation) {
        if (std::abs(rotation) < M_PI) return rotation;
        else if (rotation > M_PI) return rotation - 2 * M_PI;
        else return rotation + 2 * M_PI;
    }

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        float rotation = target.orient - kinematic.orient;
        rotation = mapToRange(rotation);
        float rotationSize = std::abs(rotation);
        float goalRotation;

        if (rotationSize < radiusOfSatisfaction) {
            goalRotation = 0;
        } else if (rotationSize > radiusOfDeceleration) {
            goalRotation = maxRotation;
        } else {
            goalRotation = maxRotation * (rotationSize / radiusOfDeceleration);
        }

        goalRotation *= rotation / std::abs(rotation);
        angular = goalRotation - kinematic.rotation;
        angular /= timeToTargetRotation;
    }
};

// Subclass for Velocity Behavior
class VelocityBehaviour : public SteeringBehaviour {
public:
    virtual void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        linear = target.vel - kinematic.vel;
        linear /= timeToTargetVelocity;
    }
};

// Subclass for Rotation Behavior
class RotationBehaviour : public SteeringBehaviour {
public:
    virtual void apply(Kinematic& kinematic, const Kinematic& target, float dtime) = 0;
};

// Add this to SteeringBehaviour.h after the other behavior classes
class WanderBehaviour : public SteeringBehaviour {
private:
    float wanderRadius = 100.0f;
    float wanderDistance = 50.0f;
    float wanderAngle = 0.0f;
    float wanderChange = 0.5f;
    const bool (*grid)[30];  // Reference to the game grid
    const int gridWidth;
    const int gridHeight;
    const int cellSize;

public:
    WanderBehaviour(const bool (*gameGrid)[30], int width, int height, int cSize) 
        : grid(gameGrid), gridWidth(width), gridHeight(height), cellSize(cSize) {}

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        // First, check for nearby walls and avoid them
        int currentGridX = static_cast<int>(kinematic.pos.x) / cellSize;
        int currentGridY = static_cast<int>(kinematic.pos.y) / cellSize;
        
        // Calculate avoidance vector
        sf::Vector2f avoidance(0, 0);
        float avoidanceStrength = 200.0f; // Increased avoidance force
        
        // Check surrounding cells for walls
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                int checkX = currentGridX + dx;
                int checkY = currentGridY + dy;
                
                if (checkX >= 0 && checkX < gridWidth && 
                    checkY >= 0 && checkY < gridHeight && 
                    grid[checkX][checkY]) {
                    
                    // Calculate vector from wall to character
                    sf::Vector2f wallCenter(checkX * cellSize + cellSize/2, 
                                         checkY * cellSize + cellSize/2);
                    sf::Vector2f toCharacter = kinematic.pos - wallCenter;
                    float distance = std::sqrt(toCharacter.x * toCharacter.x + 
                                            toCharacter.y * toCharacter.y);
                    
                    // Strong avoidance for very close walls
                    if (distance < cellSize * 2) {
                        if (distance < 0.1f) distance = 0.1f; // Prevent division by zero
                        float avoidScale = avoidanceStrength / (distance * distance);
                        avoidance += (toCharacter / distance) * avoidScale;
                    }
                }
            }
        }

        // Update wander angle with wall avoidance influence
        if (std::sqrt(avoidance.x * avoidance.x + avoidance.y * avoidance.y) > 0.1f) {
            // If there's significant avoidance force, adjust wander angle towards it
            wanderAngle = std::atan2(avoidance.y, avoidance.x);
        } else {
            // Normal wandering behavior
            wanderAngle += (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * wanderChange;
        }

        // Calculate the center of the wander circle
        sf::Vector2f circleCenter;
        if (std::abs(kinematic.vel.x) < 0.01f && std::abs(kinematic.vel.y) < 0.01f) {
            circleCenter = kinematic.pos + sf::Vector2f(
                std::cos(kinematic.orient),
                std::sin(kinematic.orient)
            ) * wanderDistance;
        } else {
            float speed = std::sqrt(kinematic.vel.x * kinematic.vel.x + kinematic.vel.y * kinematic.vel.y);
            circleCenter = kinematic.pos + (kinematic.vel / speed) * wanderDistance;
        }

        // Calculate the target point on the circle
        sf::Vector2f targetPoint = circleCenter + sf::Vector2f(
            std::cos(wanderAngle),
            std::sin(wanderAngle)
        ) * wanderRadius;

        // Combine wander force with wall avoidance
        sf::Vector2f desiredVel = targetPoint - kinematic.pos;
        float distance = std::sqrt(desiredVel.x * desiredVel.x + desiredVel.y * desiredVel.y);
        
        if (distance > 0) {
            desiredVel = (desiredVel / distance) * maxVelocity;
        }

        // Add wall avoidance to desired velocity
        desiredVel += avoidance;

        // Set steering
        linear = desiredVel - kinematic.vel;
        if (std::sqrt(linear.x * linear.x + linear.y * linear.y) > maxAcceleration) {
            float length = std::sqrt(linear.x * linear.x + linear.y * linear.y);
            linear = (linear / length) * maxAcceleration;
        }

        // Angular steering is handled in the update function
        angular = 0;
    }
};

#endif // STEERING_BEHAVIOUR_H


