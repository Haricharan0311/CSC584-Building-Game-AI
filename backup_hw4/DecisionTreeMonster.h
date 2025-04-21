#ifndef DECISION_TREE_MONSTER_H
#define DECISION_TREE_MONSTER_H

#include "MonsterInterface.h"
#include "DecisionTreeLearner.h"
#include "PatrolPoint.h"
#include "GameConstants.h"
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream>

class DecisionTreeMonster : public MonsterInterface {
private:
    sf::Vector2f pos;
    sf::Vector2f vel;
    float rotation;
    float scale;
    DecisionTreeLearner decisionTree;
    std::vector<PatrolPoint> patrolPoints;
    size_t currentPatrolIndex;
    float actionTimer;
    std::string currentAction;
    const float normalSpeed = 30.0f;
    const float chaseSpeed = 50.0f;
    const float danceTimer = 0.0f;
    sf::Vector2f lastPlayerPos;


public:
    DecisionTreeMonster(const sf::Vector2f& startPos) 
        : pos(startPos)
        , vel(0, 0)
        , rotation(0)
        , scale(1.0f)
        , currentPatrolIndex(0)
        , actionTimer(0)
        , currentAction("PATROL")
        , lastPlayerPos(0, 0) {
        
        // Train the decision tree
        decisionTree.train("monster_behavior.csv");
    }

    void update(float deltaTime, const sf::Vector2f& playerPos, const bool (*grid)[30]) {
        // Calculate player velocity
        sf::Vector2f playerVel = (playerPos - lastPlayerPos) / deltaTime;
        float playerSpeed = std::sqrt(playerVel.x * playerVel.x + playerVel.y * playerVel.y);
        lastPlayerPos = playerPos;
        

        // Create current state
        DataPoint state;
        
        // Room detection
        if (pos.x < CELL_SIZE * 20 && pos.y < CELL_SIZE * 10) 
            state.room = 1;
        else if (pos.x >= CELL_SIZE * 20 && pos.y < CELL_SIZE * 10) 
            state.room = 2;
        else 
            state.room = 3;

        // Calculate distances and angles
        sf::Vector2f toPlayer = playerPos - pos;
        state.distToPlayer = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
        state.angleToPlayer = std::atan2(toPlayer.y, toPlayer.x) * (180.0f / M_PI);
        
        // Line of sight check
        state.lineOfSight = (state.distToPlayer <= 150.0f);
        
        // Wall distance
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
        state.distToWall = minWallDist;
        
        // Patrol point distance
        if (!patrolPoints.empty()) {
            sf::Vector2f toPatrol = patrolPoints[currentPatrolIndex].position - pos;
            state.distToPatrol = std::sqrt(toPatrol.x * toPatrol.x + toPatrol.y * toPatrol.y);
        }
        
        // Other state information
        state.timeInAction = actionTimer;
        state.playerSpeed = playerSpeed;
        state.sameRoom = (static_cast<int>(playerPos.x / (CELL_SIZE * 20)) == 
                         static_cast<int>(pos.x / (CELL_SIZE * 20)));

        // Get action from decision tree
        std::string newAction = decisionTree.predict(state);
        
        // Update action timer
        if (newAction != currentAction) {
            actionTimer = 0;
            currentAction = newAction;
        } else {
            actionTimer += deltaTime;
        }
        
        // Execute action
        if (currentAction == "PATROL") {
            executePatrol(deltaTime, grid);
        } else if (currentAction == "CHASE") {
            executeChase(playerPos, deltaTime, grid);
        } else if (currentAction == "DANCE") {
            executeDance(deltaTime);
        }
        
        // Update position
        pos += vel * deltaTime;
    }

    void setupPatrolPoints(const std::vector<PatrolPoint>& points) {
        patrolPoints = points;
        currentPatrolIndex = 0;
    }

    void draw(sf::RenderWindow& window) {
        // std::cout << "Drawing monster at position: " << pos.x << ", " << pos.y << std::endl;
        // Draw monster sprite
        sf::CircleShape monsterShape(5);
        monsterShape.setFillColor(sf::Color::Blue);  // Different color from original monster
        monsterShape.setPosition(pos.x - 5, pos.y - 5);
        monsterShape.setRotation(rotation);
        monsterShape.setScale(scale, scale);
        window.draw(monsterShape);

        // Draw detection range
        sf::CircleShape detectionCircle(150.0f);
        detectionCircle.setPosition(pos.x - 150.0f, pos.y - 150.0f);
        detectionCircle.setFillColor(sf::Color::Transparent);
        detectionCircle.setOutlineColor(sf::Color(0, 0, 255, 64));
        detectionCircle.setOutlineThickness(1);
        window.draw(detectionCircle);

        // Draw current action text
        sf::Text actionText;
        sf::Font font;
        // if (font.loadFromFile("arial.ttf")) {
        //     actionText.setFont(font);
        //     actionText.setString(currentAction);
        //     actionText.setCharacterSize(12);
        //     actionText.setFillColor(sf::Color::Blue);
        //     actionText.setPosition(pos.x + 10, pos.y - 10);
        //     window.draw(actionText);
        // }
    }

    // Required MonsterInterface methods
    const sf::Vector2f& getPosition() const override { return pos; }
    void setPosition(const sf::Vector2f& p) override { pos = p; }
    void setVelocity(const sf::Vector2f& v) override { vel = v; }
    float getRotation() const override { return rotation; }
    void setRotation(float r) override { rotation = r; }
    void setScale(float s) override { scale = s; }
    void resetScale() override { scale = 1.0f; }

private:
    void executePatrol(float deltaTime, const bool (*grid)[30]) {
    if (patrolPoints.empty()) return;

    sf::Vector2f targetPos = patrolPoints[currentPatrolIndex].position;
    sf::Vector2f toTarget = targetPos - pos;
    float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

    if (distance < 5.0f) {
        currentPatrolIndex = (currentPatrolIndex + 1) % patrolPoints.size();
        return;
    }

    sf::Vector2f direction = toTarget / distance;
    sf::Vector2f avoidance(0, 0);

    int currentGridX = static_cast<int>(pos.x) / CELL_SIZE;
    int currentGridY = static_cast<int>(pos.y) / CELL_SIZE;

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int checkX = currentGridX + dx;
            int checkY = currentGridY + dy;
            if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30 && grid[checkX][checkY]) {
                avoidance.x += -dx * 20.0f;
                avoidance.y += -dy * 20.0f;
            }
        }
    }

    sf::Vector2f velocity = direction * normalSpeed + avoidance;
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > normalSpeed) {
        velocity *= (normalSpeed / speed);
    }

    vel = velocity;
}


    void executeChase(const sf::Vector2f& playerPos, float deltaTime, const bool (*grid)[30]) {
    sf::Vector2f toPlayer = playerPos - pos;
    float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    
    if (distance == 0) return;

    sf::Vector2f direction = toPlayer / distance;
    sf::Vector2f avoidance(0, 0);

    int currentGridX = static_cast<int>(pos.x) / CELL_SIZE;
    int currentGridY = static_cast<int>(pos.y) / CELL_SIZE;

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int checkX = currentGridX + dx;
            int checkY = currentGridY + dy;
            if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30 && grid[checkX][checkY]) {
                avoidance.x += -dx * 25.0f;
                avoidance.y += -dy * 25.0f;
            }
        }
    }

    sf::Vector2f velocity = direction * chaseSpeed + avoidance;
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    if (speed > chaseSpeed) {
        velocity *= (chaseSpeed / speed);
    }

    vel = velocity;
}



    // void executeChase(const sf::Vector2f& playerPos, float deltaTime) {
    //     sf::Vector2f toPlayer = playerPos - pos;
    //     float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
        
    //     if (distance < 25.0f) {
    //         vel = sf::Vector2f(0, 0);
    //         return;
    //     }
        
    //     sf::Vector2f direction = toPlayer / distance;
    //     vel = direction * chaseSpeed;
    // }

    // void executeDance(float deltaTime) {
    //     const float DANCE_DURATION = 2.0f;
    //     const float SPIN_SPEED = 360.0f;
    //     const float PULSE_SPEED = 4.0f;
        
    //     vel = sf::Vector2f(0, 0);
    //     rotation += SPIN_SPEED * deltaTime;
    //     scale = 1.0f + 0.5f * std::sin(actionTimer * PULSE_SPEED);
        
    //     if (actionTimer >= DANCE_DURATION) {
    //         rotation = 0;
    //         scale = 1.0f;
    //         currentAction = "PATROL";
    //     }
    // }
    void executeDance(float deltaTime) {
        const float DANCE_DURATION = 2.0f;
        const float SPIN_SPEED = 2.0f;
        const float PULSE_SPEED = 4.0f;
        const float MAX_SCALE = 1.5f;
        const float MIN_SCALE = 0.8f;
        const float CIRCLE_RADIUS = 10.0f;
        const float CIRCLE_SPEED = 3.0f;
        
        if (actionTimer >= DANCE_DURATION) {
            rotation = 0;
            scale = 1.0f;
            currentAction = "PATROL";
            return;
        }

        actionTimer += deltaTime;
        
        // Create a smooth circular motion
        sf::Vector2f currentPos = pos;
        sf::Vector2f circleOffset(
            std::cos(actionTimer * CIRCLE_SPEED) * CIRCLE_RADIUS,
            std::sin(actionTimer * CIRCLE_SPEED) * CIRCLE_RADIUS
        );
        pos = currentPos + circleOffset;

        // Smooth rotation
        float rotation = std::sin(actionTimer * SPIN_SPEED) * 360.0f;
        this->rotation = rotation;

        // Smooth pulsing scale effect
        float scaleProgress = (std::sin(actionTimer * PULSE_SPEED) + 1.0f) * 0.5f;
        float currentScale = MIN_SCALE + (MAX_SCALE - MIN_SCALE) * scaleProgress;
        scale = currentScale;

        // Stop any existing velocity
        vel = sf::Vector2f(0, 0);
    }
};

#endif // DECISION_TREE_MONSTER_H
