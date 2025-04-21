#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <SFML/System/Vector2.hpp>
#include <vector>
#include <memory>
#include <cmath>
#include "MonsterInterface.h"
#include "PatrolPoint.h"
#include "GameConstants.h"

enum class BehaviorStatus {
    SUCCESS,
    FAILURE,
    RUNNING
};

class MonsterBehaviorNode {
public:
    virtual ~MonsterBehaviorNode() = default;
    virtual BehaviorStatus update() = 0;
};

class MonsterSequenceNode : public MonsterBehaviorNode {
private:
    std::vector<std::unique_ptr<MonsterBehaviorNode>> children;
    size_t currentChild;

public:
    MonsterSequenceNode();
    void addChild(std::unique_ptr<MonsterBehaviorNode> child);
    BehaviorStatus update() override;
};

class MonsterSelectorNode : public MonsterBehaviorNode {
private:
    std::vector<std::unique_ptr<MonsterBehaviorNode>> children;
    size_t currentChild;

public:
    MonsterSelectorNode();
    void addChild(std::unique_ptr<MonsterBehaviorNode> child);
    BehaviorStatus update() override;
};

class MonsterActionNode : public MonsterBehaviorNode {
protected:
    MonsterInterface* monster;

public:
    MonsterActionNode(MonsterInterface* m);
};

class LineOfSightNode : public MonsterActionNode {
private:
    const bool (*grid)[30];
    const sf::Vector2f& playerPos;
    const float detectionRange;

public:
    LineOfSightNode(MonsterInterface* m, const bool (*g)[30], const sf::Vector2f& player);
    BehaviorStatus update() override;
};

class PatrolNode : public MonsterActionNode {
private:
    float deltaTime;
    const std::vector<PatrolPoint>& patrolPoints;
    size_t& currentPatrolIndex;
    const float arrivalDistance;
    const bool (*grid)[30];
    const int CELL_SIZE;

public:
    PatrolNode(MonsterInterface* m, float dt, const std::vector<PatrolPoint>& points, 
               size_t& currentIndex, const bool (*g)[30]);
    BehaviorStatus update() override;
};

class WaitNode : public MonsterActionNode {
private:
    float deltaTime;
    float& waitTimer;
    const std::vector<PatrolPoint>& patrolPoints;
    size_t& currentPatrolIndex;

public:
    WaitNode(MonsterInterface* m, float dt, float& timer, const std::vector<PatrolPoint>& points, size_t& currentIndex);
    BehaviorStatus update() override;
};

class ChaseNode : public MonsterActionNode {
private:
    const sf::Vector2f& playerPos;
    float deltaTime;
    const float chaseSpeed;
    const float catchDistance;
    const bool (*grid)[30];
    const int CELL_SIZE;

public:
    ChaseNode(MonsterInterface* m, const sf::Vector2f& player, float dt, const bool (*g)[30]);
    BehaviorStatus update() override;
};

class CollisionCheckNode : public MonsterActionNode {
private:
    const sf::Vector2f& playerPos;
    bool& hasCollided;
    const float collisionDistance;

public:
    CollisionCheckNode(MonsterInterface* m, const sf::Vector2f& player, bool& collision);
    BehaviorStatus update() override;
};

class VictoryDanceNode : public MonsterActionNode {
private:
    float deltaTime;
    float& danceTimer;
    const float DANCE_DURATION;
    const float SPIN_SPEED;
    const float PULSE_SPEED;
    const float MAX_SCALE;
    const float MIN_SCALE;
    const float CIRCLE_RADIUS;
    const float CIRCLE_SPEED;

public:
    VictoryDanceNode(MonsterInterface* m, float dt, float& timer);
    BehaviorStatus update() override;
};

class ResetPositionsNode : public MonsterActionNode {
private:
    sf::Vector2f& playerPos;
    sf::Vector2f playerStart;
    sf::Vector2f monsterStart;
    bool& hasCollided;
    const bool (*grid)[30];

public:
    ResetPositionsNode(MonsterInterface* m, sf::Vector2f& pPos, 
                      sf::Vector2f pStart, sf::Vector2f mStart, 
                      bool& collision, const bool (*g)[30]);  // Just declaration, no implementation
    
    BehaviorStatus update() override;
};

#endif // BEHAVIOR_TREE_H