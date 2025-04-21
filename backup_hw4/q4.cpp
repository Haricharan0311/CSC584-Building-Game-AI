#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "DecisionTree.h"
#include "Monster.h"
#include "GameConstants.h"

// Initialize static variables from SteeringBehaviour
float SteeringBehaviour::maxVelocity = 40.0f;
float SteeringBehaviour::maxAcceleration = 100.0f;
float SteeringBehaviour::maxAngular = 2.0f;
float SteeringBehaviour::maxRotation = 1.0f;
float SteeringBehaviour::timeToTargetVelocity = 0.3f;
float SteeringBehaviour::timeToTargetRotation = 0.3f;
float SteeringBehaviour::radiusOfSatisfaction = 8.0f;
float SteeringBehaviour::radiusOfDeceleration = 30.0f;

// Grid-based environment
const int GRID_WIDTH = 40;
const int GRID_HEIGHT = 30;
const int WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE;
const int WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE;

// Function to update kinematic based on steering
inline void update(Kinematic& character, SteeringBehaviour& steering, float deltaTime) {
    // Apply steering forces to velocity
    character.vel += steering.linear * deltaTime;
    
    // Limit velocity to maximum
    float speed = std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
    if (speed > SteeringBehaviour::maxVelocity) {
        character.vel = character.vel / speed * SteeringBehaviour::maxVelocity;
    }
    
    // Apply damping to help stop more precisely
    if (std::abs(steering.linear.x) < 0.1f && std::abs(steering.linear.y) < 0.1f) {
        // Apply damping when no acceleration is applied
        character.vel *= 0.95f;
        
        // If very slow, just stop completely
        if (speed < 1.0f) {
            character.vel = sf::Vector2f(0, 0);
        }
    }
    
    // Update position
    character.pos += character.vel * deltaTime;
    
    // DIRECT ORIENTATION TO DIRECTION OF TRAVEL
    if (speed > 5.0f) {
        // Calculate orientation from velocity
        float targetOrient = std::atan2(character.vel.y, character.vel.x);
        
        // Set orientation directly
        character.orient = targetOrient;
        
        // For debugging
        //std::cout << "Velocity: (" << character.vel.x << ", " << character.vel.y 
                  //<< "), Orient: " << (character.orient * 180.0f / M_PI) << " degrees" << std::endl;
    } else {
        // Not moving fast enough, so stop rotating
        character.rotation = 0;
    }
}

// Node for A* pathfinding
struct Node {
    int x, y;
    float g, h, f;
    Node* parent;

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
};

// Custom steering behavior for path following
class PathFollowingBehaviour : public SteeringBehaviour {
private:
    std::vector<sf::Vector2f> path;
    int currentWaypoint;
    PositionBehaviour positionBehaviour;

public:
    PathFollowingBehaviour() : currentWaypoint(0) {}

    void setPath(const std::vector<sf::Vector2f>& newPath) {
        path = newPath;
        currentWaypoint = 0;
        //std::cout << "New path set with " << path.size() << " waypoints" << std::endl;
    }

    bool isPathComplete() const {
        return path.empty() || currentWaypoint >= path.size();
    }

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        if (path.empty() || currentWaypoint >= path.size()) {
            // If we have no path or completed the path, but we're not at the final target,
            // move directly to the target position
            sf::Vector2f finalDirection = target.pos - kinematic.pos;
            float finalDistance = std::sqrt(finalDirection.x * finalDirection.x + finalDirection.y * finalDirection.y);
            
            if (finalDistance > 5.0f) {
                // We're not at the final target yet, so move directly there
                float approachSpeed = std::min(maxVelocity * (finalDistance / 30.0f), maxVelocity);
                approachSpeed = std::max(approachSpeed, 10.0f); // Minimum speed
                
                sf::Vector2f desiredVelocity = finalDirection / finalDistance * approachSpeed;
                linear = desiredVelocity - kinematic.vel;
                
                // Limit acceleration
                float currentAccel = std::sqrt(linear.x * linear.x + linear.y * linear.y);
                if (currentAccel > maxAcceleration) {
                    linear = linear / currentAccel * maxAcceleration;
                }
                
                // Calculate orientation with strong damping to prevent jitter
                if (std::abs(kinematic.vel.x) > 5.0f || std::abs(kinematic.vel.y) > 5.0f) {
                    // Only update orientation when we have significant velocity
                    float desiredOrient = std::atan2(kinematic.vel.y, kinematic.vel.x);
                    
                    // Calculate the shortest rotation to the target orientation
                    float rotDiff = desiredOrient - kinematic.orient;
                    
                    // Normalize to [-π, π]
                    while (rotDiff > M_PI) rotDiff -= 2 * M_PI;
                    while (rotDiff < -M_PI) rotDiff += 2 * M_PI;
                    
                    // Use a very gentle proportional control with a large dead zone
                    if (std::abs(rotDiff) > 0.1f) {
                        // Very gentle rotation - just 10% of the difference per second
                        angular = rotDiff * 0.5f;
                        
                        // Hard limit on angular acceleration
                        if (angular > 0.5f) angular = 0.5f;
                        if (angular < -0.5f) angular = -0.5f;
                    } else {
                        // Within dead zone - no rotation needed
                        angular = 0;
                    }
                } else {
                    // When stopped or nearly stopped, stop rotation completely
                    angular = 0;
                }
            } else {
                // We've reached the final target - apply braking force
                linear = -kinematic.vel * 5.0f; // Strong braking force
                angular = -kinematic.rotation * 5.0f; // Stop rotation
                
                // If almost stopped, set velocity to zero
                if (std::sqrt(kinematic.vel.x * kinematic.vel.x + kinematic.vel.y * kinematic.vel.y) < 1.0f) {
                    kinematic.vel = sf::Vector2f(0, 0);
                    kinematic.rotation = 0;
                    linear = sf::Vector2f(0, 0);
                    angular = 0;
                }
            }
            return;
        }

        // Get current waypoint
        sf::Vector2f waypoint = path[currentWaypoint];
        
        // Calculate direction and distance to waypoint
        sf::Vector2f direction = waypoint - kinematic.pos;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        
        // Look ahead to anticipate turns
        sf::Vector2f futurePos = kinematic.pos + kinematic.vel * 0.5f; // Look 0.5 seconds ahead
        float futureDistance = 0;
        
        if (currentWaypoint < path.size() - 1) {
            // Check if we'll overshoot the waypoint
            sf::Vector2f futureDirection = waypoint - futurePos;
            futureDistance = std::sqrt(futureDirection.x * futureDirection.x + futureDirection.y * futureDirection.y);
            
            // If we're going to overshoot or we're very close, start considering the next waypoint
            if (futureDistance < radiusOfSatisfaction || distance < radiusOfSatisfaction * 2) {
                currentWaypoint++;
                waypoint = path[currentWaypoint];
                direction = waypoint - kinematic.pos;
                distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
            }
        }
        
        // If we're at the last waypoint and close enough, stop
        if (currentWaypoint == path.size() - 1 && distance < radiusOfSatisfaction) {
            // Apply braking force
            linear = -kinematic.vel * 5.0f; // Strong braking force
            angular = -kinematic.rotation * 5.0f; // Stop rotation
            
            // If almost stopped, set velocity to zero
            if (std::sqrt(kinematic.vel.x * kinematic.vel.x + kinematic.vel.y * kinematic.vel.y) < 1.0f) {
                kinematic.vel = sf::Vector2f(0, 0);
                kinematic.rotation = 0;
                linear = sf::Vector2f(0, 0);
                angular = 0;
            }
            return;
        }

        // Calculate desired velocity
        sf::Vector2f desiredVelocity;
        if (distance > 0) {
            // Normalize direction
            desiredVelocity = direction / distance;
            
            // Adjust speed based on distance and turns
            float targetSpeed = maxVelocity;
            
            // Slow down for turns by checking angle to next waypoint
            if (currentWaypoint < path.size() - 1) {
                sf::Vector2f nextWaypoint = path[currentWaypoint + 1];
                sf::Vector2f nextDirection = nextWaypoint - waypoint;
                float nextDistance = std::sqrt(nextDirection.x * nextDirection.x + nextDirection.y * nextDirection.y);
                
                if (nextDistance > 0) {
                    nextDirection /= nextDistance;
                    
                    // Calculate dot product to determine turn sharpness
                    float dotProduct = (direction.x * nextDirection.x + direction.y * nextDirection.y) / distance;
                    
                    // Sharper turns (lower dot product) = lower speed
                    if (dotProduct < 0.7f) { // Angle greater than ~45 degrees
                        targetSpeed = maxVelocity * (0.5f + 0.5f * dotProduct);
                        targetSpeed = std::max(targetSpeed, 30.0f); // Minimum speed
                    }
                }
            }
            
            // Slow down when approaching waypoint
            if (distance < 30.0f) {
                float approachFactor = distance / 30.0f;
                float approachSpeed = maxVelocity * approachFactor;
                approachSpeed = std::max(approachSpeed, 20.0f); // Minimum speed
                targetSpeed = std::min(targetSpeed, approachSpeed);
            }
            
            // Apply the target speed
            desiredVelocity *= targetSpeed;
        } else {
            desiredVelocity = sf::Vector2f(0, 0);
        }
        
        // Calculate steering force
        linear = desiredVelocity - kinematic.vel;
        
        // Limit to maximum acceleration
        float currentAccel = std::sqrt(linear.x * linear.x + linear.y * linear.y);
        if (currentAccel > maxAcceleration) {
            linear = linear / currentAccel * maxAcceleration;
        }
        
        // Don't calculate angular steering - orientation is handled directly in update
        angular = 0;
    }
};

class Game {
private:
    sf::RenderWindow window;
    bool grid[GRID_WIDTH][GRID_HEIGHT];  // true = obstacle, false = free
    Kinematic character;
    Kinematic target;
    std::unique_ptr<DecisionNode> decisionTree;
    GameState gameState;
    std::shared_ptr<PathFollowingBehaviour> pathFollowing;
    std::shared_ptr<WanderBehaviour> wanderBehaviour;
    std::vector<sf::Vector2f> path;
    std::vector<sf::Vector2f> breadcrumbs;
    sf::Texture boidTexture;
    sf::Sprite boidSprite;
    sf::Clock clock;
    sf::Font debugFont;
    Monster monster;
    sf::Texture monsterTexture;
    std::vector<PatrolPoint> monsterPatrolPoints;

    // Create the indoor environment with rooms and obstacles
    void createEnvironment() {
        // Initialize all cells as free
        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                grid[x][y] = false;
            }
        }

        // Create outer walls
        for (int x = 0; x < GRID_WIDTH; x++) {
            grid[x][0] = true;
            grid[x][GRID_HEIGHT - 1] = true;
        }
        for (int y = 0; y < GRID_HEIGHT; y++) {
            grid[0][y] = true;
            grid[GRID_WIDTH - 1][y] = true;
        }

        // Room 1 (top left)
        for (int x = 10; x < 20; x++) {
            grid[x][10] = true;
        }
        for (int y = 0; y < 10; y++) {
            grid[20][y] = true;
        }
        // Obstacle in room 1
        for (int x = 5; x < 8; x++) {
            for (int y = 5; y < 8; y++) {
                grid[x][y] = true;
            }
        }

        // Room 2 (top right)
        for (int y = 0; y < 10; y++) {
            grid[30][y] = true;
        }
        // Obstacle in room 2
        for (int x = 25; x < 28; x++) {
            for (int y = 5; y < 8; y++) {
                grid[x][y] = true;
            }
        }

        // Room 3 (bottom)
        for (int x = 10; x < 30; x++) {
            grid[x][20] = true;
        }
        for (int y = 10; y < 20; y++) {
            grid[10][y] = true;
            grid[30][y] = true;
        }
        for (int i=20;i<29;i++){
            grid[i][10] = true;
        }
        // Doorways
        grid[15][10] = false;
        grid[16][10] = false;
        grid[24][10] = false;
        grid[25][10] = false;
        grid[20][5] = false;
        grid[20][6] = false;
        grid[30][5] = false;
        grid[30][6] = false;
        grid[15][20] = false;
        grid[16][20] = false;
        grid[20][20]=false;
        grid[25][20] = false;
        grid[26][20] = false;
        grid[30][20] = true;

        // Obstacle in room 3
        for (int x = 18; x < 22; x++) {
            for (int y = 14; y < 18; y++) {
                grid[x][y] = true;
            }
        }
    }

    // A* pathfinding algorithm
    std::vector<sf::Vector2f> findPath(int startX, int startY, int goalX, int goalY) {
        std::vector<sf::Vector2f> resultPath;
        
        std::cout << "findPath called with start=(" << startX << "," << startY 
                  << "), goal=(" << goalX << "," << goalY << ")" << std::endl;
        
        // Check if start or goal is an obstacle
        if (startX < 0 || startX >= GRID_WIDTH || startY < 0 || startY >= GRID_HEIGHT) {
            std::cout << "Start position out of bounds" << std::endl;
            return resultPath;
        }
        if (goalX < 0 || goalX >= GRID_WIDTH || goalY < 0 || goalY >= GRID_HEIGHT) {
            std::cout << "Goal position out of bounds" << std::endl;
            return resultPath;
        }
        if (grid[startX][startY]) {
            std::cout << "Start position is an obstacle" << std::endl;
            return resultPath;
        }
        if (grid[goalX][goalY]) {
            std::cout << "Goal position is an obstacle" << std::endl;
            return resultPath;
        }

        // Define movement directions (8-way movement)
        const int dx[8] = {-1, 0, 1, 0, -1, -1, 1, 1};
        const int dy[8] = {0, -1, 0, 1, -1, 1, -1, 1};

        // Open and closed sets
        std::vector<Node*> openSet;
        std::vector<Node*> closedSet;

        // Create start and goal nodes
        Node* startNode = new Node(startX, startY);
        Node* goalNode = new Node(goalX, goalY);

        // Add start node to open set
        openSet.push_back(startNode);

        while (!openSet.empty()) {
            // Find node with lowest f score
            int currentIndex = 0;
            for (int i = 1; i < openSet.size(); i++) {
                if (openSet[i]->f < openSet[currentIndex]->f) {
                    currentIndex = i;
                }
            }

            Node* current = openSet[currentIndex];

            // Check if we reached the goal
            if (current->x == goalNode->x && current->y == goalNode->y) {
                // Reconstruct path
                Node* currentPtr = current;
                while (currentPtr != nullptr) {
                    resultPath.push_back(sf::Vector2f(
                        currentPtr->x * CELL_SIZE + CELL_SIZE / 2,
                        currentPtr->y * CELL_SIZE + CELL_SIZE / 2
                    ));
                    currentPtr = currentPtr->parent;
                }
                std::reverse(resultPath.begin(), resultPath.end());
                
                // Clean up nodes
                for (auto node : openSet) {
                    if (node != current) delete node;
                }
                for (auto node : closedSet) {
                    delete node;
                }
                delete current;
                delete goalNode;
                
                return resultPath;
            }

            // Move current node from open to closed set
            openSet.erase(openSet.begin() + currentIndex);
            closedSet.push_back(current);

            // Check all neighbors
            for (int i = 0; i < 8; i++) {
                int newX = current->x + dx[i];
                int newY = current->y + dy[i];

                // Check if valid position
                if (newX < 0 || newX >= GRID_WIDTH || newY < 0 || newY >= GRID_HEIGHT || grid[newX][newY]) {
                    continue;
                }

                // Check if in closed set
                bool inClosedSet = false;
                for (auto node : closedSet) {
                    if (node->x == newX && node->y == newY) {
                        inClosedSet = true;
                        break;
                    }
                }
                if (inClosedSet) continue;

                // Calculate g, h, and f scores
                float tentativeG = current->g + ((i < 4) ? 1.0f : 1.414f);  // Diagonal movement costs more
                
                // Check if this path is better or if neighbor is not in open set
                bool inOpenSet = false;
                Node* neighborNode = nullptr;
                for (auto node : openSet) {
                    if (node->x == newX && node->y == newY) {
                        inOpenSet = true;
                        neighborNode = node;
                        break;
                    }
                }

                if (!inOpenSet) {
                    // Create new node
                    neighborNode = new Node(newX, newY);
                    neighborNode->g = tentativeG;
                    neighborNode->h = std::sqrt(std::pow(goalX - newX, 2) + std::pow(goalY - newY, 2));
                    neighborNode->f = neighborNode->g + neighborNode->h;
                    neighborNode->parent = current;
                    openSet.push_back(neighborNode);
                } else if (tentativeG < neighborNode->g) {
                    // Update existing node
                    neighborNode->g = tentativeG;
                    neighborNode->f = neighborNode->g + neighborNode->h;
                    neighborNode->parent = current;
                }
            }
        }

        // Clean up nodes
        for (auto node : openSet) {
            delete node;
        }
        for (auto node : closedSet) {
            delete node;
        }
        delete goalNode;
        
        return resultPath;  // Return empty path if no path found
    }

    // Function to print the grid state
    void printGridState() {
        //std::cout << "Grid state:" << std::endl;
        for (int y = 0; y < 10; y++) {
            for (int x = 0; x < 10; x++) {
                //std::cout << (grid[x][y] ? "X" : ".");
            }
            //std::cout << std::endl;
        }
    }

    void setupDecisionTree() {
        // Create behaviors
        pathFollowing = std::make_shared<PathFollowingBehaviour>();
        wanderBehaviour = std::make_shared<WanderBehaviour>(grid, GRID_WIDTH, GRID_HEIGHT, CELL_SIZE);

        // Create a more complex decision tree
        decisionTree = std::make_unique<TimeInStateNode>(
            5.0f, // Change behavior every 5 seconds
            std::make_unique<WallProximityNode>(
                30.0f, // Wall distance threshold
                std::make_unique<BehaviorNode>(pathFollowing), // Too close to wall -> PathFollow to room center
                std::make_unique<VelocityNode>( // Far from wall -> Check velocity
                    35.0f, // Velocity threshold
                    std::make_unique<BehaviorNode>(wanderBehaviour), // High speed -> Wander
                    std::make_unique<BehaviorNode>(pathFollowing) // Low speed -> PathFollow
                )
            ),
            std::make_unique<WallProximityNode>(
                30.0f,
                std::make_unique<BehaviorNode>(pathFollowing),
                std::make_unique<BehaviorNode>(wanderBehaviour)
            )
        );

        if (!debugFont.loadFromFile("arial.ttf")) {
            std::cerr << "Failed to load font!" << std::endl;
        }
    }

    std::string getBehaviorName(std::shared_ptr<SteeringBehaviour> behavior) {
        if (!behavior) return "None";
        if (dynamic_cast<WanderBehaviour*>(behavior.get())) return "Wander";
        if (dynamic_cast<PathFollowingBehaviour*>(behavior.get())) return "PathFollow";
        return "Unknown";
    }

    int getCurrentRoom() {
        if (gameState.isInRoom(1)) return 1;
        if (gameState.isInRoom(2)) return 2;
        if (gameState.isInRoom(3)) return 3;
        return 0;
    }

    void generateNewRandomTarget() {
        int attempts = 0;
        const int MAX_ATTEMPTS = 5;

        while (attempts < MAX_ATTEMPTS) {
            // Pick a random room
            int targetRoom = (rand() % 3) + 1;
            sf::Vector2f roomCenter = gameState.getRoomCenter(targetRoom);
            
            // Random offset from room center (smaller radius)
            float randX = (rand() % 40 - 20) * 1.5f;  // Even smaller range
            float randY = (rand() % 40 - 20) * 1.5f;
            sf::Vector2f newTarget = roomCenter + sf::Vector2f(randX, randY);
            
            int targetX = static_cast<int>(newTarget.x) / CELL_SIZE;
            int targetY = static_cast<int>(newTarget.y) / CELL_SIZE;
            
            // Check if target is valid
            if (targetX >= 0 && targetX < GRID_WIDTH && 
                targetY >= 0 && targetY < GRID_HEIGHT && 
                !grid[targetX][targetY]) {
                
                int startX = static_cast<int>(character.pos.x) / CELL_SIZE;
                int startY = static_cast<int>(character.pos.y) / CELL_SIZE;
                
                path = findPath(startX, startY, targetX, targetY);
                if (!path.empty()) {
                    pathFollowing->setPath(path);
                    target.pos = newTarget;
                    return;
                }
            }
            attempts++;
        }

        // If all attempts failed, just use room center
        int currentRoom = getCurrentRoom();
        sf::Vector2f roomCenter = gameState.getRoomCenter(currentRoom);
        int startX = static_cast<int>(character.pos.x) / CELL_SIZE;
        int startY = static_cast<int>(character.pos.y) / CELL_SIZE;
        int targetX = static_cast<int>(roomCenter.x) / CELL_SIZE;
        int targetY = static_cast<int>(roomCenter.y) / CELL_SIZE;
        
        path = findPath(startX, startY, targetX, targetY);
        if (!path.empty()) {
            pathFollowing->setPath(path);
            target.pos = roomCenter;
        }
    }

    void setupMonster() {
        // Load monster texture
        if (!monsterTexture.loadFromFile("monster.png")) {
            std::cerr << "Failed to load monster texture!" << std::endl;
            monsterTexture = boidTexture;
        }

        // Set up patrol points in clear areas of each room
        monsterPatrolPoints = {
            PatrolPoint(sf::Vector2f(CELL_SIZE * 15, CELL_SIZE * 5)),  // Room 1 (top left), away from obstacle
            PatrolPoint(sf::Vector2f(CELL_SIZE * 35, CELL_SIZE * 5)),  // Room 2 (top right), away from obstacle
            PatrolPoint(sf::Vector2f(CELL_SIZE * 15, CELL_SIZE * 15)), // Room 3 (bottom), left side
            PatrolPoint(sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 15))  // Room 3 (bottom), right side
        };
        
        monster.setupPatrolPoints(monsterPatrolPoints);
        monster.startRecording("monster_behavior_dummy.csv");
    }

public:
    Game() : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Indoor Pathfinding"),
             gameState(character, grid, GRID_WIDTH, GRID_HEIGHT, CELL_SIZE),
             monster(sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 5)) {  // Start in room 2
        createEnvironment();
        setupMonster();
        
        // Load boid texture
        if (!boidTexture.loadFromFile("boid-sm.png")) {
            std::cerr << "Failed to load boid texture!" << std::endl;
        }
        boidSprite.setTexture(boidTexture);
        boidSprite.setOrigin(boidTexture.getSize().x / 2.0f, boidTexture.getSize().y / 2.0f);
        
        // Initialize character
        character.pos = sf::Vector2f(CELL_SIZE * 3, CELL_SIZE * 3);
        character.vel = sf::Vector2f(0, 0);
        character.orient = 0;
        character.rotation = 0;
        
        // Initialize target
        target.pos = character.pos;
        target.vel = sf::Vector2f(0, 0);
        target.orient = 0;
        target.rotation = 0;

        // Setup decision tree
        setupDecisionTree();
    }

    void run() {
        // Initialize with path following behavior and a random target
        generateNewRandomTarget();
        gameState.setBehavior(pathFollowing);  // Set initial behavior
        bool forcedPathFollow = true;
        float pathFollowTimer = 0.0f;

        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
            }
            
            float deltaTime = clock.restart().asSeconds();
            
            // Update state time
            gameState.updateStateTime(deltaTime);

            // Get current behavior
            auto currentBehavior = gameState.getCurrentBehavior();

            // Handle behavior transitions
            if (forcedPathFollow) {
                pathFollowTimer += deltaTime;
                
                // Check if current path is complete
                if (pathFollowing->isPathComplete() || path.empty()) {
                    // Generate new path to different room
                    int currentRoom = getCurrentRoom();
                    int nextRoom = ((currentRoom % 3) + 1);
                    sf::Vector2f roomCenter = gameState.getRoomCenter(nextRoom);
                    
                    // Random offset from room center
                    float randX = (rand() % 40 - 20) * 1.5f;
                    float randY = (rand() % 40 - 20) * 1.5f;
                    sf::Vector2f newTarget = roomCenter + sf::Vector2f(randX, randY);
                    
                    // Find path to new target
                    int startX = static_cast<int>(character.pos.x) / CELL_SIZE;
                    int startY = static_cast<int>(character.pos.y) / CELL_SIZE;
                    int targetX = static_cast<int>(newTarget.x) / CELL_SIZE;
                    int targetY = static_cast<int>(newTarget.y) / CELL_SIZE;
                    
                    path = findPath(startX, startY, targetX, targetY);
                    if (!path.empty()) {
                        pathFollowing->setPath(path);
                        target.pos = newTarget;
                    }
                }

                // Check if we should switch to wandering
                if (pathFollowTimer > 8.0f) {
                    forcedPathFollow = false;
                    gameState.setBehavior(wanderBehaviour);
                    gameState.resetStateTime();
                }
            } else {
                // In wandering state
                if (gameState.getTimeInCurrentState() > 4.0f) {
                    forcedPathFollow = true;
                    pathFollowTimer = 0.0f;
                    generateNewRandomTarget();
                    gameState.setBehavior(pathFollowing);
                    gameState.resetStateTime();
                }
            }

            // Before applying current behavior, check if monster has collided
            if (monster.hasCollidedWithPlayer()) {
                // Stop the player's movement
                character.vel = sf::Vector2f(0, 0);
            } else {
                // Apply current behavior only if not collided
                if (currentBehavior) {
                    currentBehavior->apply(character, target, deltaTime);
                    update(character, *currentBehavior, deltaTime);
                }
            }
            
            // Add breadcrumb every few frames
            static int frameCount = 0;
            if (frameCount++ % 20 == 0) { // Add breadcrumb every 10 frames
                breadcrumbs.push_back(character.pos);
                // Limit number of breadcrumbs to prevent performance issues
                if (breadcrumbs.size() > 100) {
                    breadcrumbs.erase(breadcrumbs.begin());
                }
            }
            
            // Update sprite orientation
            boidSprite.setPosition(character.pos);
            boidSprite.setRotation(character.orient * 180.0f / M_PI);
            
            // Debug print orientation
            if (frameCount++ % 60 == 0) { // Print every 60 frames
                //std::cout << "Orientation: " << character.orient << " radians, " 
                          //<< (character.orient * 180.0f / M_PI) << " degrees" << std::endl;
                //std::cout << "Velocity: (" << character.vel.x << ", " << character.vel.y << ")" << std::endl;
                //std::cout << "Speed: " << std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y) << std::endl;
            }
            
            // Update monster
            monster.update(deltaTime, character.pos, grid);

            // Render
            window.clear(sf::Color::White);
            
            // Draw grid
            for (int x = 0; x < GRID_WIDTH; x++) {
                for (int y = 0; y < GRID_HEIGHT; y++) {
                    sf::RectangleShape cell(sf::Vector2f(CELL_SIZE, CELL_SIZE));
                    cell.setPosition(x * CELL_SIZE, y * CELL_SIZE);
                    cell.setOutlineThickness(1);
                    cell.setOutlineColor(sf::Color(200, 200, 200));
                    
                    if (grid[x][y]) {
                        cell.setFillColor(sf::Color::Black);
                    } else {
                        cell.setFillColor(sf::Color::White);
                    }
                    
                    window.draw(cell);
                }
            }
            
            // Draw path
            for (const auto& point : path) {
                sf::CircleShape pathPoint(3);
                pathPoint.setFillColor(sf::Color::Green);
                pathPoint.setPosition(point.x - 3, point.y - 3);
                window.draw(pathPoint);
            }
            
            // Draw breadcrumbs
            sf::CircleShape breadcrumb(2);
            breadcrumb.setFillColor(sf::Color(0, 0, 255, 128)); // Blue with transparency
            for (const auto& pos : breadcrumbs) {
                breadcrumb.setPosition(pos.x - 2, pos.y - 2); // Center the circle on the position
                window.draw(breadcrumb);
            }
            
            // Draw target
            sf::CircleShape targetShape(5);
            targetShape.setFillColor(sf::Color::Red);
            targetShape.setPosition(target.pos.x - 5, target.pos.y - 5);
            window.draw(targetShape);
            
            // Draw character
            window.draw(boidSprite);

            // Draw monster
            monster.draw(window, monsterTexture);

            // Draw patrol points
            for (const auto& point : monsterPatrolPoints) {
                sf::CircleShape patrolPoint(5);
                patrolPoint.setFillColor(sf::Color::Yellow);
                patrolPoint.setPosition(point.position.x - 5, point.position.y - 5);
                window.draw(patrolPoint);
            }

            // Draw line between monster and current patrol point
            if (!monsterPatrolPoints.empty()) {
                sf::Vertex line[] = {
                    sf::Vertex(monster.getPosition(), sf::Color::Yellow),
                    sf::Vertex(monsterPatrolPoints[monster.getCurrentPatrolIndex()].position, sf::Color::Yellow)
                };
                window.draw(line, 2, sf::Lines);
            }

            // Debug Information
            sf::Text debugText;
            debugText.setFont(debugFont);
            debugText.setCharacterSize(14);
            debugText.setFillColor(sf::Color::Black);

            // Update debug info to include monster state
            std::string debugInfo = 
                "Current Behavior: " + getBehaviorName(gameState.getCurrentBehavior()) + "\n" +
                "Wall Distance: " + std::to_string(gameState.getDistanceToNearestWall()) + "\n" +
                "Speed: " + std::to_string(gameState.getCurrentSpeed()) + "\n" +
                "Room: " + std::to_string(getCurrentRoom()) + "\n" +
                "Monster Position: (" + std::to_string(monster.getPosition().x) + ", " + 
                               std::to_string(monster.getPosition().y) + ")\n" +
                "Monster Patrol Point: " + std::to_string(monster.getCurrentPatrolIndex()) + "\n" +
                "Distance to Player: " + std::to_string(
                    std::sqrt(std::pow(monster.getPosition().x - character.pos.x, 2) +
                             std::pow(monster.getPosition().y - character.pos.y, 2))) + "\n";

            debugText.setString(debugInfo);
            debugText.setPosition(10, 10);
            window.draw(debugText);

            // Draw wall distance indicator
            float wallDist = gameState.getDistanceToNearestWall();
            sf::CircleShape distanceIndicator(wallDist);
            distanceIndicator.setPosition(character.pos.x - wallDist, character.pos.y - wallDist);
            distanceIndicator.setFillColor(sf::Color::Transparent);
            distanceIndicator.setOutlineColor(sf::Color(0, 255, 0, 64));  // Semi-transparent green
            distanceIndicator.setOutlineThickness(1);
            window.draw(distanceIndicator);

            // Draw velocity vector
            // sf::RectangleShape velocityLine(sf::Vector2f(gameState.getCurrentSpeed(), 2));
            // velocityLine.setPosition(character.pos);
            // velocityLine.setRotation(std::atan2(character.vel.y, character.vel.x) * 180 / M_PI);
            // velocityLine.setFillColor(sf::Color::Blue);
            // window.draw(velocityLine);

            window.display();
        }
    }

    ~Game() {
        monster.stopRecording();
    }
};

int main() {
    Game game;
    game.run();
    return 0;
}