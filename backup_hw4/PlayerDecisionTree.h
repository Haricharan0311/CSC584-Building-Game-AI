#ifndef PLAYER_DECISION_TREE_H
#define PLAYER_DECISION_TREE_H

#include <SFML/System/Vector2.hpp>
#include <memory>
#include "SteeringBehaviour.h"
#include "GameState.h"
#include "DecisionTreeLearner.h"

// Forward declarations
class GameState;

class PlayerDecisionNode {
public:
    virtual ~PlayerDecisionNode() = default;
    virtual bool evaluate(const GameState& state) = 0;
};

class PlayerBehaviorNode : public PlayerDecisionNode {
private:
    std::shared_ptr<SteeringBehaviour> behavior;

public:
    PlayerBehaviorNode(std::shared_ptr<SteeringBehaviour> b) : behavior(b) {}
    
    bool evaluate(const GameState& state) override {
        state.setBehavior(behavior);
        return true;
    }
};

class PlayerVelocityNode : public PlayerDecisionNode {
private:
    float threshold;
    std::unique_ptr<PlayerDecisionNode> trueNode;
    std::unique_ptr<PlayerDecisionNode> falseNode;

public:
    PlayerVelocityNode(float t, std::unique_ptr<PlayerDecisionNode> tNode, 
                      std::unique_ptr<PlayerDecisionNode> fNode)
        : threshold(t)
        , trueNode(std::move(tNode))
        , falseNode(std::move(fNode)) {}

    bool evaluate(const GameState& state) override {
        if (state.getCurrentSpeed() > threshold) {
            return trueNode->evaluate(state);
        }
        return falseNode->evaluate(state);
    }
};

class PlayerWallProximityNode : public PlayerDecisionNode {
private:
    float threshold;
    std::unique_ptr<PlayerDecisionNode> tooCloseNode;
    std::unique_ptr<PlayerDecisionNode> farEnoughNode;

public:
    PlayerWallProximityNode(float t, std::unique_ptr<PlayerDecisionNode> close, 
                           std::unique_ptr<PlayerDecisionNode> far)
        : threshold(t)
        , tooCloseNode(std::move(close))
        , farEnoughNode(std::move(far)) {}

    bool evaluate(const GameState& state) override {
        if (state.getDistanceToNearestWall() < threshold) {
            return tooCloseNode->evaluate(state);
        }
        return farEnoughNode->evaluate(state);
    }
};

class PlayerTimeInStateNode : public PlayerDecisionNode {
private:
    float timeThreshold;
    std::unique_ptr<PlayerDecisionNode> timeExceededNode;
    std::unique_ptr<PlayerDecisionNode> withinTimeNode;

public:
    PlayerTimeInStateNode(float t, std::unique_ptr<PlayerDecisionNode> exceeded, 
                         std::unique_ptr<PlayerDecisionNode> within)
        : timeThreshold(t)
        , timeExceededNode(std::move(exceeded))
        , withinTimeNode(std::move(within)) {}

    bool evaluate(const GameState& state) override {
        if (state.getTimeInCurrentState() > timeThreshold) {
            return timeExceededNode->evaluate(state);
        }
        return withinTimeNode->evaluate(state);
    }
};

class LearnedBehaviorNode {
private:
    sf::Vector2f currentVelocity;
    float timeInCurrentAction;
    
    std::string makeDecision(float playerSpeed, float angleToPlayer, 
                            float distToPlayer, float timeInAction) {
        // Implementing the exact decision tree from training
        if (playerSpeed <= 0.0f) {
            if (angleToPlayer <= -178.01f) {
                if (timeInAction <= 0.56f) {
                    return "DANCE";
                } else {
                    return "CHASE";
                }
            } else {
                if (timeInAction <= 4.05f) {
                    if (distToPlayer <= 440.95f) {
                        return "DANCE";
                    } else {
                        return "PATROL";
                    }
                } else {
                    return "CHASE";
                }
            }
        } else {
            if (distToPlayer <= 149.99f) {
                return "CHASE";
            } else {
                return "PATROL";
            }
        }
    }
    
public:
    LearnedBehaviorNode() : currentVelocity(0, 0), timeInCurrentAction(0) {}

    void update(const sf::Vector2f& playerPos, const sf::Vector2f& monsterPos, 
                float deltaTime, const bool (*grid)[30]) {
        // Update time in current action
        timeInCurrentAction += deltaTime;

        // Calculate features
        sf::Vector2f toMonster = monsterPos - playerPos;
        float distToMonster = std::sqrt(toMonster.x * toMonster.x + toMonster.y * toMonster.y);
        float angleToMonster = std::atan2(toMonster.y, toMonster.x) * 180.0f / M_PI; // Convert to degrees
        float speed = std::sqrt(currentVelocity.x * currentVelocity.x + 
                              currentVelocity.y * currentVelocity.y);

        // Get decision from tree
        std::string action = makeDecision(speed, angleToMonster, distToMonster, timeInCurrentAction);
        
        // Convert decision to velocity
        sf::Vector2f newVelocity;
        if (action == "CHASE") {
            // Move towards monster at high speed
            if (distToMonster > 0) {
                newVelocity = (toMonster / distToMonster) * 50.0f;
            }
        }
        else if (action == "PATROL") {
            // Move at moderate speed
            if (distToMonster > 0) {
                newVelocity = (toMonster / distToMonster) * 30.0f;
            }
        }
        else if (action == "DANCE") {
            // Stop moving
            newVelocity = sf::Vector2f(0, 0);
        }

        // If the action has changed, reset the timer
        if ((currentVelocity.x == 0 && newVelocity.x != 0) ||
            (currentVelocity.y == 0 && newVelocity.y != 0) ||
            (currentVelocity.x != 0 && newVelocity.x == 0) ||
            (currentVelocity.y != 0 && newVelocity.y == 0)) {
            timeInCurrentAction = 0;
        }

        currentVelocity = newVelocity;
    }

    sf::Vector2f getVelocity() const {
        return currentVelocity;
    }
};

#endif // PLAYER_DECISION_TREE_H
