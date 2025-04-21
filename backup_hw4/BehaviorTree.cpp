#include "BehaviorTree.h"
#include "Monster.h"
#include <cmath>
#include <iostream>
#include <algorithm>

// Add this at the top of BehaviorTree.cpp, after the includes
sf::Vector2f normalize(const sf::Vector2f& v) {
    float length = std::sqrt(v.x * v.x + v.y * v.y);
    if (length > 0) {
        return v / length;
    }
    return v;
}

// MonsterSequenceNode implementations
MonsterSequenceNode::MonsterSequenceNode() : currentChild(0) {}

void MonsterSequenceNode::addChild(std::unique_ptr<MonsterBehaviorNode> child) {
    children.push_back(std::move(child));
}

BehaviorStatus MonsterSequenceNode::update() {
    // If we've completed all children, reset and return success
    if (currentChild >= children.size()) {
        currentChild = 0;
        return BehaviorStatus::SUCCESS;
    }

    // Update the current child
    BehaviorStatus childStatus = children[currentChild]->update();

    switch (childStatus) {
        case BehaviorStatus::SUCCESS:
            // Move to next child
            currentChild++;
            // If we still have children, return RUNNING
            if (currentChild < children.size()) {
                return BehaviorStatus::RUNNING;
            }
            // Otherwise, reset and return SUCCESS
            currentChild = 0;
            return BehaviorStatus::SUCCESS;

        case BehaviorStatus::FAILURE:
            // Reset sequence and return failure
            currentChild = 0;
            return BehaviorStatus::FAILURE;

        case BehaviorStatus::RUNNING:
            return BehaviorStatus::RUNNING;
    }

    return BehaviorStatus::FAILURE;
}

// MonsterSelectorNode implementations
MonsterSelectorNode::MonsterSelectorNode() : currentChild(0) {}

void MonsterSelectorNode::addChild(std::unique_ptr<MonsterBehaviorNode> child) {
    children.push_back(std::move(child));
}

BehaviorStatus MonsterSelectorNode::update() {
    // Try each child in order until one succeeds
    for (size_t i = 0; i < children.size(); i++) {
        BehaviorStatus status = children[i]->update();
        
        switch (status) {
            case BehaviorStatus::SUCCESS:
                // Child succeeded, reset current child and return success
                currentChild = i;  // Remember which child succeeded
                return BehaviorStatus::SUCCESS;
                
            case BehaviorStatus::RUNNING:
                // Child is still running, keep using this child
                currentChild = i;
                return BehaviorStatus::RUNNING;
                
            case BehaviorStatus::FAILURE:
                // Try next child
                continue;
        }
    }
    
    // All children failed
    currentChild = children.size() - 1;  // Default to last child (patrol)
    return BehaviorStatus::FAILURE;
}

// MonsterActionNode implementation
MonsterActionNode::MonsterActionNode(MonsterInterface* m) : monster(m) {}

// LineOfSightNode implementations
LineOfSightNode::LineOfSightNode(MonsterInterface* m, const bool (*g)[30], const sf::Vector2f& player)
    : MonsterActionNode(m), grid(g), playerPos(player), detectionRange(300.0f) {}

BehaviorStatus LineOfSightNode::update() {
    sf::Vector2f monsterPos = monster->getPosition();
    sf::Vector2f toPlayer = playerPos - monsterPos;
    float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    
    // First check if player is within detection range
    if (distance <= detectionRange) {
        std::cout << "Player in detection range at distance: " << distance << std::endl;
        
        // Implement Bresenham's line algorithm to check for walls
        int x1 = static_cast<int>(monsterPos.x) / CELL_SIZE;
        int y1 = static_cast<int>(monsterPos.y) / CELL_SIZE;
        int x2 = static_cast<int>(playerPos.x) / CELL_SIZE;
        int y2 = static_cast<int>(playerPos.y) / CELL_SIZE;
        
        bool hasLineOfSight = true;  // Assume we can see until proven otherwise
        
        // Simple line of sight check
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int x = x1;
        int y = y1;
        
        int stepX = (x1 < x2) ? 1 : -1;
        int stepY = (y1 < y2) ? 1 : -1;
        
        int error = dx - dy;
        
        while (x != x2 || y != y2) {
            if (x >= 0 && x < 40 && y >= 0 && y < 30) {
                if (grid[x][y]) {
                    std::cout << "Wall blocking line of sight at (" << x << "," << y << ")" << std::endl;
                    hasLineOfSight = false;
                    break;
                }
            }
            
            int error2 = error * 2;
            
            if (error2 > -dy) {
                error -= dy;
                x += stepX;
            }
            if (error2 < dx) {
                error += dx;
                y += stepY;
            }
        }
        
        if (hasLineOfSight) {
            std::cout << "Clear line of sight to player! Starting chase!" << std::endl;
            return BehaviorStatus::SUCCESS;
        }
    }
    
    return BehaviorStatus::FAILURE;
}

// PatrolNode implementations
PatrolNode::PatrolNode(MonsterInterface* m, float dt, const std::vector<PatrolPoint>& points, 
                      size_t& currentIndex, const bool (*g)[30])
    : MonsterActionNode(m), deltaTime(dt), patrolPoints(points), 
      currentPatrolIndex(currentIndex), arrivalDistance(5.0f), grid(g), CELL_SIZE(20) {}

BehaviorStatus PatrolNode::update() {
    if (patrolPoints.empty()) return BehaviorStatus::FAILURE;
    
    sf::Vector2f targetPos = patrolPoints[currentPatrolIndex].position;
    sf::Vector2f monsterPos = monster->getPosition();
    sf::Vector2f toTarget = targetPos - monsterPos;
    float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
    
    if (distance < arrivalDistance) {
        return BehaviorStatus::SUCCESS;
    }
    
    // Basic direction to target
    sf::Vector2f direction = toTarget / distance;
    
    // Simple wall avoidance
    sf::Vector2f avoidance(0, 0);
    int currentGridX = static_cast<int>(monsterPos.x) / CELL_SIZE;
    int currentGridY = static_cast<int>(monsterPos.y) / CELL_SIZE;
    
    // Check only immediate adjacent cells
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;  // Skip current cell
            
            int checkX = currentGridX + dx;
            int checkY = currentGridY + dy;
            
            if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30 && grid[checkX][checkY]) {
                // Add a small avoidance force away from the wall
                avoidance.x += -dx * 20.0f;
                avoidance.y += -dy * 20.0f;
            }
        }
    }
    
    // Combine movement with gentle wall avoidance
    sf::Vector2f finalVelocity = direction * 30.0f + avoidance;
    
    // Normalize if speed is too high
    float speed = std::sqrt(finalVelocity.x * finalVelocity.x + finalVelocity.y * finalVelocity.y);
    if (speed > 30.0f) {
        finalVelocity *= (30.0f / speed);
    }
    
    monster->setVelocity(finalVelocity);
    return BehaviorStatus::RUNNING;
}

// WaitNode implementations
WaitNode::WaitNode(MonsterInterface* m, float dt, float& timer, const std::vector<PatrolPoint>& points, size_t& currentIndex)
    : MonsterActionNode(m), deltaTime(dt), waitTimer(timer), patrolPoints(points), currentPatrolIndex(currentIndex) {}

BehaviorStatus WaitNode::update() {
    if (patrolPoints.empty()) return BehaviorStatus::FAILURE;
    
    monster->setVelocity(sf::Vector2f(0, 0));
    waitTimer += deltaTime;
    
    if (waitTimer >= patrolPoints[currentPatrolIndex].waitTime) {
        waitTimer = 0;
        currentPatrolIndex = (currentPatrolIndex + 1) % patrolPoints.size();
        return BehaviorStatus::SUCCESS;
    }
    
    return BehaviorStatus::RUNNING;
}

// ChaseNode implementations
ChaseNode::ChaseNode(MonsterInterface* m, const sf::Vector2f& player, float dt, const bool (*g)[30])
    : MonsterActionNode(m), playerPos(player), deltaTime(dt), 
      chaseSpeed(50.0f), catchDistance(15.0f), grid(g), CELL_SIZE(20) {}

BehaviorStatus ChaseNode::update() {
    sf::Vector2f monsterPos = monster->getPosition();
    sf::Vector2f toPlayer = playerPos - monsterPos;
    float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    
    if (distance < catchDistance) {
        monster->setVelocity(sf::Vector2f(0, 0));
        return BehaviorStatus::SUCCESS;
    }
    
    // Basic direction to player
    sf::Vector2f direction = toPlayer / distance;
    
    // Simple wall avoidance
    sf::Vector2f avoidance(0, 0);
    int currentGridX = static_cast<int>(monsterPos.x) / CELL_SIZE;
    int currentGridY = static_cast<int>(monsterPos.y) / CELL_SIZE;
    
    // Check only immediate adjacent cells
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;  // Skip current cell
            
            int checkX = currentGridX + dx;
            int checkY = currentGridY + dy;
            
            if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30 && grid[checkX][checkY]) {
                // Add a small avoidance force away from the wall
                avoidance.x += -dx * 25.0f;
                avoidance.y += -dy * 25.0f;
            }
        }
    }
    
    // Combine movement with gentle wall avoidance
    sf::Vector2f finalVelocity = direction * chaseSpeed + avoidance;
    
    // Normalize if speed is too high
    float speed = std::sqrt(finalVelocity.x * finalVelocity.x + finalVelocity.y * finalVelocity.y);
    if (speed > chaseSpeed) {
        finalVelocity *= (chaseSpeed / speed);
    }
    
    monster->setVelocity(finalVelocity);
    return BehaviorStatus::RUNNING;
}

// CollisionCheckNode implementations
CollisionCheckNode::CollisionCheckNode(MonsterInterface* m, const sf::Vector2f& player, bool& collision)
    : MonsterActionNode(m), playerPos(player), hasCollided(collision), collisionDistance(25.0f) {}

BehaviorStatus CollisionCheckNode::update() {
    sf::Vector2f monsterPos = monster->getPosition();
    sf::Vector2f toPlayer = playerPos - monsterPos;
    float distance = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    
    if (distance < collisionDistance) {
        hasCollided = true;  // Set collision state
        std::cout << "Collision check success! Distance: " << distance << std::endl;
        return BehaviorStatus::SUCCESS;
    }
    
    return BehaviorStatus::FAILURE;
}

// VictoryDanceNode implementations
VictoryDanceNode::VictoryDanceNode(MonsterInterface* m, float dt, float& timer)
    : MonsterActionNode(m)
    , deltaTime(dt)
    , danceTimer(timer)
    , DANCE_DURATION(2.0f)
    , SPIN_SPEED(2.0f)
    , PULSE_SPEED(4.0f)
    , MAX_SCALE(1.5f)
    , MIN_SCALE(0.8f)
    , CIRCLE_RADIUS(10.0f)
    , CIRCLE_SPEED(3.0f)
{}

BehaviorStatus VictoryDanceNode::update() {
    if (danceTimer >= DANCE_DURATION) {
        monster->resetScale();
        monster->setRotation(0);
        danceTimer = 0;
        return BehaviorStatus::SUCCESS;
    }

    danceTimer += deltaTime;
    
    // Create a smooth circular motion
    sf::Vector2f currentPos = monster->getPosition();
    sf::Vector2f circleOffset(
        std::cos(danceTimer * CIRCLE_SPEED) * CIRCLE_RADIUS,
        std::sin(danceTimer * CIRCLE_SPEED) * CIRCLE_RADIUS
    );
    monster->setPosition(currentPos + circleOffset);

    // Smooth rotation
    float rotation = std::sin(danceTimer * SPIN_SPEED) * 360.0f;
    monster->setRotation(rotation);

    // Smooth pulsing scale effect
    float scaleProgress = (std::sin(danceTimer * PULSE_SPEED) + 1.0f) * 0.5f;
    float currentScale = MIN_SCALE + (MAX_SCALE - MIN_SCALE) * scaleProgress;
    monster->setScale(currentScale);

    // Stop any existing velocity
    monster->setVelocity(sf::Vector2f(0, 0));

    return BehaviorStatus::RUNNING;
}

// ResetPositionsNode implementations
ResetPositionsNode::ResetPositionsNode(MonsterInterface* m, sf::Vector2f& pPos, 
                                     sf::Vector2f pStart, sf::Vector2f mStart, 
                                     bool& collision, const bool (*g)[30])
    : MonsterActionNode(m), playerPos(pPos), playerStart(pStart), 
      monsterStart(mStart), hasCollided(collision), grid(g) {}

BehaviorStatus ResetPositionsNode::update() {
    std::cout << "Resetting positions..." << std::endl;
    
    // Get monster's future position (monsterStart)
    sf::Vector2f monsterPos = monsterStart;
    const float MIN_SPAWN_DISTANCE = 200.0f;  // Minimum distance from monster to spawn player
    
    bool validPosition = false;
    sf::Vector2f newPlayerPos;
    const float SAFE_DISTANCE = CELL_SIZE * 1.5f;
    
    int attempts = 0;
    const int MAX_ATTEMPTS = 100;
    
    while (!validPosition && attempts < MAX_ATTEMPTS) {
        attempts++;
        // Try rooms in order of distance from monster
        std::vector<int> rooms = {1, 2, 3};
        std::sort(rooms.begin(), rooms.end(), [&](int a, int b) {
            sf::Vector2f centerA, centerB;
            switch(a) {
                case 1: centerA = sf::Vector2f(10 * CELL_SIZE, 5 * CELL_SIZE); break;
                case 2: centerA = sf::Vector2f(25 * CELL_SIZE, 5 * CELL_SIZE); break;
                case 3: centerA = sf::Vector2f(20 * CELL_SIZE, 15 * CELL_SIZE); break;
            }
            switch(b) {
                case 1: centerB = sf::Vector2f(10 * CELL_SIZE, 5 * CELL_SIZE); break;
                case 2: centerB = sf::Vector2f(25 * CELL_SIZE, 5 * CELL_SIZE); break;
                case 3: centerB = sf::Vector2f(20 * CELL_SIZE, 15 * CELL_SIZE); break;
            }
            
            float distA = std::sqrt(std::pow(centerA.x - monsterPos.x, 2) + 
                                  std::pow(centerA.y - monsterPos.y, 2));
            float distB = std::sqrt(std::pow(centerB.x - monsterPos.x, 2) + 
                                  std::pow(centerB.y - monsterPos.y, 2));
            return distA > distB;  // Sort by descending distance
        });
        
        // Try each room starting with the furthest
        for (int room : rooms) {
            float x, y;
            switch(room) {
                case 1: // Top left room
                    x = (rand() % 15 + 3) * CELL_SIZE;
                    y = (rand() % 7 + 2) * CELL_SIZE;
                    break;
                case 2: // Top right room
                    x = (rand() % 7 + 22) * CELL_SIZE;
                    y = (rand() % 7 + 2) * CELL_SIZE;
                    break;
                case 3: // Bottom room
                    x = (rand() % 17 + 12) * CELL_SIZE;
                    y = (rand() % 7 + 12) * CELL_SIZE;
                    break;
            }
            
            // Check distance from monster
            float distToMonster = std::sqrt(std::pow(x - monsterPos.x, 2) + 
                                          std::pow(y - monsterPos.y, 2));
            if (distToMonster < MIN_SPAWN_DISTANCE) {
                continue;  // Too close to monster, try next room
            }
            
            // Check if position is valid
            int gridX = static_cast<int>(x) / CELL_SIZE;
            int gridY = static_cast<int>(y) / CELL_SIZE;
            
            if (gridX >= 0 && gridX < 40 && gridY >= 0 && gridY < 30 && !grid[gridX][gridY]) {
                bool tooCloseToWall = false;
                
                // Check surrounding cells for walls
                for (int dx = -2; dx <= 2; dx++) {
                    for (int dy = -2; dy <= 2; dy++) {
                        int checkX = gridX + dx;
                        int checkY = gridY + dy;
                        
                        if (checkX >= 0 && checkX < 40 && checkY >= 0 && checkY < 30) {
                            if (grid[checkX][checkY]) {
                                sf::Vector2f wallCenter(
                                    checkX * CELL_SIZE + CELL_SIZE/2,
                                    checkY * CELL_SIZE + CELL_SIZE/2
                                );
                                sf::Vector2f toWall = sf::Vector2f(x, y) - wallCenter;
                                float distance = std::sqrt(
                                    toWall.x * toWall.x + toWall.y * toWall.y
                                );
                                
                                if (distance < SAFE_DISTANCE) {
                                    tooCloseToWall = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (tooCloseToWall) break;
                }
                
                if (!tooCloseToWall) {
                    newPlayerPos = sf::Vector2f(x, y);
                    validPosition = true;
                    break;
                }
            }
        }
        
        if (validPosition) break;
    }
    
    // If we still couldn't find a valid position, use the furthest room center
    if (!validPosition) {
        sf::Vector2f roomCenters[3] = {
            sf::Vector2f(10 * CELL_SIZE, 5 * CELL_SIZE),   // Room 1
            sf::Vector2f(25 * CELL_SIZE, 5 * CELL_SIZE),   // Room 2
            sf::Vector2f(20 * CELL_SIZE, 15 * CELL_SIZE)   // Room 3
        };
        
        float maxDistance = 0;
        for (const auto& center : roomCenters) {
            sf::Vector2f toMonster = center - monsterPos;
            float distance = std::sqrt(toMonster.x * toMonster.x + toMonster.y * toMonster.y);
            if (distance > maxDistance) {
                maxDistance = distance;
                newPlayerPos = center;
            }
        }
    }
    
    // Reset positions
    playerPos = newPlayerPos;
    monster->setPosition(monsterStart);
    monster->setVelocity(sf::Vector2f(0, 0));
    
    // Reset collision state
    hasCollided = false;
    
    std::cout << "Player respawned at: (" << playerPos.x << ", " << playerPos.y 
              << ") after " << attempts << " attempts" << std::endl;
    std::cout << "Distance from monster: " << 
        std::sqrt(std::pow(playerPos.x - monsterStart.x, 2) + 
                 std::pow(playerPos.y - monsterStart.y, 2)) << std::endl;
    
    return BehaviorStatus::SUCCESS;
}

