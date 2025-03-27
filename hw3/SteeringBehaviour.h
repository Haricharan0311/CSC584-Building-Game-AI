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

#endif // STEERING_BEHAVIOUR_H


