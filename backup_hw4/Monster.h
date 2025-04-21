#ifndef MONSTER_H
#define MONSTER_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>
#include "MonsterInterface.h"
#include "PatrolPoint.h"
#include "BehaviorTree.h"
#include "GameConstants.h"
#include <fstream>
#include <sstream>
#include <iomanip>

class Monster : public MonsterInterface {
private:
    sf::Vector2f pos;
    sf::Vector2f vel;
    float rotation;
    float scale;
    std::unique_ptr<MonsterBehaviorNode> behaviorTree;
    std::vector<PatrolPoint> patrolPoints;
    size_t currentPatrolIndex;
    float waitTimer;
    float danceTimer;
    bool hasCollided;
    const float detectionRange = 150.0f;
    const float normalSpeed = 30.0f;
    const float chaseSpeed = 50.0f;
    const float DANCE_DURATION = 2.0f;

    struct MonsterState {
        int currentRoom;
        float distanceToPlayer;
        bool hasLineOfSight;
        float distanceToNearestWall;
        std::string currentAction;
        float timeInCurrentAction;
        bool playerInSameRoom;
        float distanceToPatrolPoint;
        float angleToPlayer;
        float playerSpeed;
        sf::Vector2f playerVelocity;
        
        // Constructor with new features
        MonsterState() 
            : currentRoom(0)
            , distanceToPlayer(0)
            , hasLineOfSight(false)
            , distanceToNearestWall(0)
            , currentAction("PATROL")
            , timeInCurrentAction(0)
            , playerInSameRoom(false)
            , distanceToPatrolPoint(0)
            , angleToPlayer(0)
            , playerSpeed(0)
            , playerVelocity(sf::Vector2f(0, 0)) {}
        
        // Updated CSV conversion
        std::string toCSV() const {
            std::stringstream ss;
            ss << currentRoom << ","
               << std::fixed << std::setprecision(2) << distanceToPlayer << ","
               << (hasLineOfSight ? "1" : "0") << ","
               << std::fixed << std::setprecision(2) << distanceToNearestWall << ","
               << std::fixed << std::setprecision(2) << timeInCurrentAction << ","
               << (playerInSameRoom ? "1" : "0") << ","
               << std::fixed << std::setprecision(2) << distanceToPatrolPoint << ","
               << std::fixed << std::setprecision(2) << angleToPlayer << ","
               << std::fixed << std::setprecision(2) << playerSpeed << ","
               << currentAction;
            return ss.str();
        }
    };
    
    std::ofstream dataLog;
    float stateTimer;
    MonsterState currentState;
    std::string lastAction;
    float actionTimer;

public:
    Monster(const sf::Vector2f& startPos) 
        : pos(startPos)
        , vel(0, 0)
        , rotation(0)
        , scale(1.0f)
        , currentPatrolIndex(0)
        , waitTimer(0)
        , danceTimer(0)
        , hasCollided(false) {}

    void setupPatrolPoints(const std::vector<PatrolPoint>& points) {
        patrolPoints = points;
    }

    void setupBehaviorTree(const sf::Vector2f& playerPos, const bool (*grid)[30]) {
        // Create root selector
        auto root = std::make_unique<MonsterSelectorNode>();

        // Create collision/victory sequence (highest priority)
        auto victorySequence = std::make_unique<MonsterSequenceNode>();
        victorySequence->addChild(std::make_unique<CollisionCheckNode>(this, playerPos, hasCollided));
        victorySequence->addChild(std::make_unique<VictoryDanceNode>(this, 0.016f, danceTimer));
        victorySequence->addChild(std::make_unique<ResetPositionsNode>(
            this, 
            const_cast<sf::Vector2f&>(playerPos), 
            sf::Vector2f(60, 60),
            sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 5),
            hasCollided,
            grid
        ));

        // Create chase sequence (second priority)
        auto chaseSequence = std::make_unique<MonsterSequenceNode>();
        chaseSequence->addChild(std::make_unique<LineOfSightNode>(this, grid, playerPos));
        chaseSequence->addChild(std::make_unique<ChaseNode>(this, playerPos, 0.016f, grid));

        // Create patrol sequence (lowest priority)
        auto patrolSequence = std::make_unique<MonsterSequenceNode>();
        patrolSequence->addChild(std::make_unique<PatrolNode>(this, 0.016f, patrolPoints, currentPatrolIndex, grid));
        patrolSequence->addChild(std::make_unique<WaitNode>(this, 0.016f, waitTimer, patrolPoints, currentPatrolIndex));

        // Add sequences to root in priority order
        root->addChild(std::move(victorySequence));  // Victory sequence MUST be first
        root->addChild(std::move(chaseSequence));
        root->addChild(std::move(patrolSequence));

        behaviorTree = std::move(root);
    }

    void update(float deltaTime, const sf::Vector2f& playerPos, const bool (*grid)[30]) {
        if (!behaviorTree) {
            setupBehaviorTree(playerPos, grid);
        }

        // Calculate player velocity (you'll need to store previous player position)
        static sf::Vector2f lastPlayerPos = playerPos;
        sf::Vector2f playerVel = (playerPos - lastPlayerPos) / deltaTime;
        lastPlayerPos = playerPos;

        // Update and record state
        if (dataLog.is_open()) {
            updateState(playerPos, playerVel, grid, deltaTime);
            dataLog << currentState.toCSV() << std::endl;
        }

        // Debug output for collision detection
        sf::Vector2f toPlayer = playerPos - pos;
        float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
        
        // Update behavior tree
        BehaviorStatus status = behaviorTree->update();
        
        // Debug the behavior tree status
        if (hasCollided) {
            //std::cout << "Has collided is true, behavior status: " << 
                //(status == BehaviorStatus::SUCCESS ? "SUCCESS" : 
                 //status == BehaviorStatus::FAILURE ? "FAILURE" : "RUNNING") << 
                //" Dance Timer: " << danceTimer << std::endl;
        }

        // Only update position if not in victory dance
        if (!hasCollided || danceTimer >= DANCE_DURATION) {
            pos += vel * deltaTime;
        }
    }

    void draw(sf::RenderWindow& window, const sf::Texture& texture) {
        // Draw monster sprite (make it smaller)
        sf::CircleShape monsterShape(5);  // Changed from 10 to 5
        monsterShape.setFillColor(sf::Color::Red);
        monsterShape.setPosition(pos.x - 5, pos.y - 5);  // Adjust position offset for smaller size
        monsterShape.setRotation(rotation);
        monsterShape.setScale(scale, scale);
        window.draw(monsterShape);

        // Draw detection range (outer circle)
        // sf::CircleShape detectionCircle(detectionRange);
        // detectionCircle.setPosition(pos.x - detectionRange, pos.y - detectionRange);
        // detectionCircle.setFillColor(sf::Color::Transparent);
        // detectionCircle.setOutlineColor(sf::Color(255, 0, 0, 64));
        // detectionCircle.setOutlineThickness(1);
        // window.draw(detectionCircle);

        // Draw collision radius (inner circle)
        // sf::CircleShape collisionCircle(25.0f);  // Match the collision distance
        // collisionCircle.setPosition(pos.x - 25.0f, pos.y - 25.0f);
        // collisionCircle.setFillColor(sf::Color::Transparent);
        // collisionCircle.setOutlineColor(sf::Color(255, 255, 0, 128));  // Yellow, semi-transparent
        // collisionCircle.setOutlineThickness(1);
        // window.draw(collisionCircle);
    }

    // Getters and setters
    const sf::Vector2f& getPosition() const { return pos; }
    void setPosition(const sf::Vector2f& p) { pos = p; }
    void setVelocity(const sf::Vector2f& v) { vel = v; }
    float getRotation() const { return rotation; }
    void setRotation(float r) { rotation = r; }
    void setScale(float s) { scale = s; }
    void resetScale() { scale = 1.0f; }
    size_t getCurrentPatrolIndex() const { return currentPatrolIndex; }
    bool hasCollidedWithPlayer() const { return hasCollided; }

    void startRecording(const std::string& filename) {
        dataLog.open(filename);
        if (dataLog.is_open()) {
            // Updated CSV header with new columns
            dataLog << "Room,DistToPlayer,LineOfSight,DistToWall,TimeInAction,"
                   << "SameRoom,DistToPatrol,AngleToPlayer,PlayerSpeed,Action\n";
        }
        actionTimer = 0.0f;
        lastAction = "PATROL";
    }

    void stopRecording() {
        if (dataLog.is_open()) {
            dataLog.close();
        }
    }

private:
    void updateState(const sf::Vector2f& playerPos, const sf::Vector2f& playerVel, const bool (*grid)[30], float deltaTime) {
        // Update existing state data
        // Room detection
        if (pos.x < CELL_SIZE * 20 && pos.y < CELL_SIZE * 10) 
            currentState.currentRoom = 1;
        else if (pos.x >= CELL_SIZE * 20 && pos.y < CELL_SIZE * 10) 
            currentState.currentRoom = 2;
        else 
            currentState.currentRoom = 3;
        
        // Get player's room
        int playerRoom;
        if (playerPos.x < CELL_SIZE * 20 && playerPos.y < CELL_SIZE * 10) 
            playerRoom = 1;
        else if (playerPos.x >= CELL_SIZE * 20 && playerPos.y < CELL_SIZE * 10) 
            playerRoom = 2;
        else 
            playerRoom = 3;
        
        // Update same room status
        currentState.playerInSameRoom = (currentState.currentRoom == playerRoom);
        
        // Calculate distance and angle to player
        sf::Vector2f toPlayer = playerPos - pos;
        currentState.distanceToPlayer = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
        currentState.angleToPlayer = std::atan2(toPlayer.y, toPlayer.x) * (180.0f / M_PI);
        
        // Line of sight check
        currentState.hasLineOfSight = (currentState.distanceToPlayer <= detectionRange);
        
        // Calculate distance to nearest wall (existing code)
        float minWallDist = 1000.0f;
        int currentGridX = static_cast<int>(pos.x) / CELL_SIZE;
        int currentGridY = static_cast<int>(pos.y) / CELL_SIZE;
        
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                int checkX = currentGridX + dx;
                int checkY = currentGridY + dy;
                
                if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30 && grid[checkX][checkY]) {
                    sf::Vector2f wallCenter(checkX * CELL_SIZE + CELL_SIZE/2, checkY * CELL_SIZE + CELL_SIZE/2);
                    sf::Vector2f toWall = pos - wallCenter;
                    float distance = std::sqrt(toWall.x * toWall.x + toWall.y * toWall.y);
                    minWallDist = std::min(minWallDist, distance);
                }
            }
        }
        currentState.distanceToNearestWall = minWallDist;
        
        // Calculate distance to current patrol point
        if (!patrolPoints.empty()) {
            sf::Vector2f toPatrol = patrolPoints[currentPatrolIndex].position - pos;
            currentState.distanceToPatrolPoint = std::sqrt(toPatrol.x * toPatrol.x + toPatrol.y * toPatrol.y);
        }
        
        // Update player velocity and speed
        currentState.playerVelocity = playerVel;
        currentState.playerSpeed = std::sqrt(playerVel.x * playerVel.x + playerVel.y * playerVel.y);
        
        // Update current action and time in action
        std::string currentAction;
        if (hasCollided) {
            currentAction = "DANCE";
        } else if (currentState.hasLineOfSight) {
            currentAction = "CHASE";
        } else {
            currentAction = "PATROL";
        }
        
        // Update action timer
        if (currentAction != lastAction) {
            actionTimer = 0.0f;
            lastAction = currentAction;
        } else {
            actionTimer += deltaTime;
        }
        
        currentState.currentAction = currentAction;
        currentState.timeInCurrentAction = actionTimer;
    }
};

#endif // MONSTER_H
