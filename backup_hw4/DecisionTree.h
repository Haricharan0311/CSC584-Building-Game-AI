#ifndef DECISION_TREE_H
#define DECISION_TREE_H

#include <memory>
#include <SFML/System/Vector2.hpp>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include <limits>

// Forward declarations
class GameState;

// Base class for all decision tree nodes
class DecisionNode {
public:
    virtual bool evaluate(const GameState& state) = 0;
    virtual ~DecisionNode() = default;
};

// GameState class to track environment parameters
class GameState {
private:
    mutable std::shared_ptr<SteeringBehaviour> currentBehavior;
    mutable float timeInCurrentBehavior;  // Track how long we've been in current behavior
    mutable sf::Vector2f currentTarget;   // Track current movement target

public:
    Kinematic& character;
    const bool (*grid)[30];  // Reference to the game grid
    const int gridWidth;
    const int gridHeight;
    const int cellSize;

    GameState(Kinematic& char_, const bool (*gameGrid)[30], int width, int height, int cSize)
        : character(char_)
        , grid(gameGrid)
        , gridWidth(width)
        , gridHeight(height)
        , cellSize(cSize)
        , timeInCurrentBehavior(0)
        , currentTarget(char_.pos) {}

    void setBehavior(std::shared_ptr<SteeringBehaviour> behavior) const {
        currentBehavior = behavior;
    }

    std::shared_ptr<SteeringBehaviour> getCurrentBehavior() const {
        return currentBehavior;
    }

    float getCurrentSpeed() const {
        return std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
    }

    float getDistanceToNearestWall() const {
        int gridX = static_cast<int>(character.pos.x) / cellSize;
        int gridY = static_cast<int>(character.pos.y) / cellSize;
        
        float minDistance = std::numeric_limits<float>::max();
        
        // Check surrounding cells in a 5x5 area
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                int checkX = gridX + dx;
                int checkY = gridY + dy;
                
                if (checkX >= 0 && checkX < gridWidth && 
                    checkY >= 0 && checkY < gridHeight && 
                    grid[checkX][checkY]) {
                        
                    float dist = std::sqrt(
                        std::pow((checkX * cellSize + cellSize/2) - character.pos.x, 2) +
                        std::pow((checkY * cellSize + cellSize/2) - character.pos.y, 2)
                    );
                    minDistance = std::min(minDistance, dist);
                }
            }
        }
        
        return minDistance;
    }

    bool isInRoom(int roomNumber) const {
        int gridX = static_cast<int>(character.pos.x) / cellSize;
        int gridY = static_cast<int>(character.pos.y) / cellSize;
        
        // Define room boundaries
        switch(roomNumber) {
            case 1: // Top left room
                return gridX < 20 && gridY < 10;
            case 2: // Top right room
                return gridX >= 20 && gridX < 30 && gridY < 10;
            case 3: // Bottom room
                return gridX >= 10 && gridX < 30 && gridY >= 10 && gridY < 20;
            default:
                return false;
        }
    }

    sf::Vector2f getRoomCenter(int roomNumber) const {
        switch(roomNumber) {
            case 1: // Top left room
                return sf::Vector2f(10 * cellSize, 5 * cellSize);
            case 2: // Top right room
                return sf::Vector2f(25 * cellSize, 5 * cellSize);
            case 3: // Bottom room
                return sf::Vector2f(20 * cellSize, 15 * cellSize);
            default:
                return sf::Vector2f(0, 0);
        }
    }

    float getDistanceFromRoomCenter() const {
        sf::Vector2f center;
        if (isInRoom(1)) center = getRoomCenter(1);
        else if (isInRoom(2)) center = getRoomCenter(2);
        else if (isInRoom(3)) center = getRoomCenter(3);
        else return 0;
        
        return std::sqrt(
            std::pow(center.x - character.pos.x, 2) +
            std::pow(center.y - character.pos.y, 2)
        );
    }

    void updateStateTime(float deltaTime) const {
        timeInCurrentBehavior += deltaTime;
    }

    float getTimeInCurrentState() const {
        return timeInCurrentBehavior;
    }

    void resetStateTime() const {
        timeInCurrentBehavior = 0;
    }

    void setTarget(sf::Vector2f target) const {
        currentTarget = target;
    }

    sf::Vector2f getCurrentTarget() const {
        return currentTarget;
    }
};

// Behavior node (leaf node)
class BehaviorNode : public DecisionNode {
private:
    std::shared_ptr<SteeringBehaviour> behavior;

public:
    BehaviorNode(std::shared_ptr<SteeringBehaviour> b) : behavior(b) {}
    
    bool evaluate(const GameState& state) override {
        state.setBehavior(behavior);
        return true;
    }
};

// Decision node for checking velocity
class VelocityNode : public DecisionNode {
private:
    float threshold;
    std::unique_ptr<DecisionNode> trueNode;
    std::unique_ptr<DecisionNode> falseNode;

public:
    VelocityNode(float t, std::unique_ptr<DecisionNode> tNode, std::unique_ptr<DecisionNode> fNode)
        : threshold(t)
        , trueNode(std::move(tNode))
        , falseNode(std::move(fNode)) {}
    
    bool evaluate(const GameState& state) override {
        float currentSpeed = state.getCurrentSpeed();
        if (currentSpeed > threshold) {
            return trueNode->evaluate(state);
        }
        return falseNode->evaluate(state);
    }
};

// Decision node for checking distance to wall
class WallProximityNode : public DecisionNode {
private:
    float threshold;
    std::unique_ptr<DecisionNode> tooCloseNode;
    std::unique_ptr<DecisionNode> farEnoughNode;

public:
    WallProximityNode(float t, std::unique_ptr<DecisionNode> close, std::unique_ptr<DecisionNode> far)
        : threshold(t)
        , tooCloseNode(std::move(close))
        , farEnoughNode(std::move(far)) {}
    
    bool evaluate(const GameState& state) override {
        float wallDistance = state.getDistanceToNearestWall();
        if (wallDistance < threshold) {
            return tooCloseNode->evaluate(state);
        }
        return farEnoughNode->evaluate(state);
    }
};

// Decision node for checking room location
class RoomLocationNode : public DecisionNode {
private:
    int roomNumber;
    std::unique_ptr<DecisionNode> inRoomNode;
    std::unique_ptr<DecisionNode> notInRoomNode;

public:
    RoomLocationNode(int room, std::unique_ptr<DecisionNode> in, std::unique_ptr<DecisionNode> out)
        : roomNumber(room)
        , inRoomNode(std::move(in))
        , notInRoomNode(std::move(out)) {}
    
    bool evaluate(const GameState& state) override {
        if (state.isInRoom(roomNumber)) {
            return inRoomNode->evaluate(state);
        }
        return notInRoomNode->evaluate(state);
    }
};

// Decision node for checking time in state
class TimeInStateNode : public DecisionNode {
private:
    float threshold;
    std::unique_ptr<DecisionNode> timeExceededNode;
    std::unique_ptr<DecisionNode> withinTimeNode;

public:
    TimeInStateNode(float t, std::unique_ptr<DecisionNode> exceeded, std::unique_ptr<DecisionNode> within)
        : threshold(t)
        , timeExceededNode(std::move(exceeded))
        , withinTimeNode(std::move(within)) {}
    
    bool evaluate(const GameState& state) override {
        if (state.getTimeInCurrentState() > threshold) {
            state.resetStateTime();
            return timeExceededNode->evaluate(state);
        }
        return withinTimeNode->evaluate(state);
    }
};

#endif // DECISION_TREE_H
