#include <SFML/Graphics.hpp>
#include "DecisionTreeMonster.h"
#include "GameConstants.h"
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "PlayerDecisionTree.h"
#include <vector>
#include <iostream>
#include <filesystem>
#include <queue>
#include <cmath>
#include <unordered_map>

const int GRID_WIDTH = 40;
const int GRID_HEIGHT = 30;
const int WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE;   // 800 pixels
const int WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE;  // 600 pixels

// Initialize static variables from SteeringBehaviour if not already done
float SteeringBehaviour::maxVelocity = 40.0f;
float SteeringBehaviour::maxAcceleration = 100.0f;
float SteeringBehaviour::maxAngular = 2.0f;
float SteeringBehaviour::maxRotation = 1.0f;
float SteeringBehaviour::timeToTargetVelocity = 0.3f;
float SteeringBehaviour::timeToTargetRotation = 0.3f;
float SteeringBehaviour::radiusOfSatisfaction = 8.0f;
float SteeringBehaviour::radiusOfDeceleration = 30.0f;

// Node for A* pathfinding
struct Node {
    int x, y;
    float g, h, f;
    Node* parent;

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
};

// PathFollowingBehaviour class
class PathFollowingBehaviour : public SteeringBehaviour {
private:
    std::vector<sf::Vector2f> path;
    size_t currentWaypoint;

public:
    PathFollowingBehaviour() : currentWaypoint(0) {}

    void setPath(const std::vector<sf::Vector2f>& newPath) {
        path = newPath;
        currentWaypoint = 0;
    }

    bool isPathComplete() const {
        return path.empty() || currentWaypoint >= path.size();
    }

    void apply(Kinematic& character, const Kinematic& target, float deltaTime) override {
        if (path.empty() || currentWaypoint >= path.size()) {
            linear = sf::Vector2f(0, 0);
            angular = 0;
            return;
        }

        sf::Vector2f currentTarget = path[currentWaypoint];
        sf::Vector2f toTarget = currentTarget - character.pos;
        float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

        if (distance < radiusOfSatisfaction) {
            currentWaypoint++;
            if (currentWaypoint >= path.size()) {
                linear = sf::Vector2f(0, 0);
                angular = 0;
                return;
            }
            currentTarget = path[currentWaypoint];
            toTarget = currentTarget - character.pos;
            distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);
        }

        float targetSpeed = maxVelocity;
        if (distance < radiusOfDeceleration) {
            targetSpeed = maxVelocity * (distance / radiusOfDeceleration);
        }

        sf::Vector2f desiredVelocity = toTarget / distance * targetSpeed;
        linear = desiredVelocity - character.vel;
        
        if (std::sqrt(linear.x * linear.x + linear.y * linear.y) > maxAcceleration) {
            float length = std::sqrt(linear.x * linear.x + linear.y * linear.y);
            linear = linear / length * maxAcceleration;
        }

        angular = 0;  // Let the update function handle orientation
    }
};

void update(Kinematic& character, SteeringBehaviour& steering, float deltaTime) {
    // Apply steering forces to velocity
    character.vel += steering.linear * deltaTime;
    
    // Limit velocity to maximum
    float speed = std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
    if (speed > SteeringBehaviour::maxVelocity) {
        character.vel = character.vel / speed * SteeringBehaviour::maxVelocity;
    }
    
    // Apply damping
    if (std::abs(steering.linear.x) < 0.1f && std::abs(steering.linear.y) < 0.1f) {
        character.vel *= 0.95f;
        if (speed < 1.0f) {
            character.vel = sf::Vector2f(0, 0);
        }
    }
    
    // Update position
    character.pos += character.vel * deltaTime;
    
    // Update orientation based on velocity
    if (speed > 5.0f) {
        character.orient = std::atan2(character.vel.y, character.vel.x);
    }
}

class GameWithDTMonster {
private:
    sf::RenderWindow window;
    bool grid[GRID_WIDTH][GRID_HEIGHT];
    DecisionTreeMonster dtMonster;
    std::vector<PatrolPoint> patrolPoints;
    sf::Font font;
    
    Kinematic character;
    std::shared_ptr<PathFollowingBehaviour> pathFollowing;
    std::shared_ptr<WanderBehaviour> wanderBehaviour;
    std::shared_ptr<SteeringBehaviour> currentBehavior;
    std::vector<sf::Vector2f> path;
    float behaviorTimer;
    bool forcedPathFollow;
    bool playerCollided;
    sf::Vector2f playerStartPos;  // Add this to store initial spawn position

    // Breadcrumb tracking
    std::vector<sf::Vector2f> playerBreadcrumbs;
    std::vector<sf::Vector2f> monsterBreadcrumbs;
    const float BREADCRUMB_INTERVAL = 0.5f;  // Time between breadcrumbs in seconds
    float playerBreadcrumbTimer;
    float monsterBreadcrumbTimer;
    const int MAX_BREADCRUMBS = 50;  // Maximum number of breadcrumbs to keep

    sf::Texture boidTexture;
sf::Sprite boidSprite;

    void updateBreadcrumbs(float deltaTime) {
        // Update player breadcrumbs
        playerBreadcrumbTimer += deltaTime;
        if (playerBreadcrumbTimer >= BREADCRUMB_INTERVAL) {
            playerBreadcrumbs.push_back(character.pos);
            if (playerBreadcrumbs.size() > MAX_BREADCRUMBS) {
                playerBreadcrumbs.erase(playerBreadcrumbs.begin());
            }
            playerBreadcrumbTimer = 0;
        }

        // Update monster breadcrumbs
        monsterBreadcrumbTimer += deltaTime;
        if (monsterBreadcrumbTimer >= BREADCRUMB_INTERVAL) {
            monsterBreadcrumbs.push_back(dtMonster.getPosition());
            if (monsterBreadcrumbs.size() > MAX_BREADCRUMBS) {
                monsterBreadcrumbs.erase(monsterBreadcrumbs.begin());
            }
            monsterBreadcrumbTimer = 0;
        }
    }

    void drawBreadcrumbs() {
        // Draw player breadcrumbs (blue)
        for (const auto& pos : playerBreadcrumbs) {
            sf::CircleShape crumb(3);
            crumb.setFillColor(sf::Color(0, 0, 255, 128));  // Semi-transparent blue
            crumb.setPosition(pos.x - 3, pos.y - 3);
            window.draw(crumb);
        }

        // Draw monster breadcrumbs (red)
        for (const auto& pos : monsterBreadcrumbs) {
            sf::CircleShape crumb(3);
            crumb.setFillColor(sf::Color(255, 0, 0, 128));  // Semi-transparent red
            crumb.setPosition(pos.x - 3, pos.y - 3);
            window.draw(crumb);
        }
    }

    void respawnPlayer() {
        // Reset player position to starting point
        character.pos = playerStartPos;
        character.vel = sf::Vector2f(0, 0);
        character.orient = 0;
        character.rotation = 0;
        playerCollided = false;  // Reset collision state
        
        // Reset monster position
        dtMonster.setPosition(sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 5));
        
        // Clear breadcrumbs on respawn
        playerBreadcrumbs.clear();
        monsterBreadcrumbs.clear();
    }

public:
    GameWithDTMonster() 
        : dtMonster(sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 5)),
          playerCollided(false),
          playerStartPos(CELL_SIZE * 3, CELL_SIZE * 3),
          behaviorTimer(0),
          forcedPathFollow(true),
          playerBreadcrumbTimer(0),
          monsterBreadcrumbTimer(0)
    {
        // Create window settings
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;

        // Create the window with explicit style flags
        window.create(
            sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
            "Decision Tree Monster",
            sf::Style::Default,
            settings
        );

        if (!window.isOpen()) {
            throw std::runtime_error("Failed to create window!");
        }

        // Set frame rate limit instead of vsync
        window.setFramerateLimit(60);

        std::cout << "Window created with size: " << WINDOW_WIDTH << "x" << WINDOW_HEIGHT << std::endl;
        std::cout << "Actual window size: " << window.getSize().x << "x" << window.getSize().y << std::endl;

        // Initialize other components
        createEnvironment();
        setupPatrolPoints();
        if (boidTexture.loadFromFile("boid-sm.png")) {
    boidSprite.setTexture(boidTexture);
    boidSprite.setOrigin(boidTexture.getSize().x / 2.0f, boidTexture.getSize().y / 2.0f);
    boidSprite.setScale(1.0f, 1.0f); // Adjust size as needed
} 
else{
    std::cerr << "Failed to load boid.png for decision tree monster!" << std::endl;
}
        character.pos = playerStartPos;
        character.vel = sf::Vector2f(0, 0);
        character.orient = 0;
        character.rotation = 0;
        
        if (!font.loadFromFile("arial.ttf")) {
            std::cout << "Note: Failed to load font (this is okay)" << std::endl;
        }

        // Initialize behaviors
        pathFollowing = std::make_shared<PathFollowingBehaviour>();
        wanderBehaviour = std::make_shared<WanderBehaviour>(grid, GRID_WIDTH, GRID_HEIGHT, CELL_SIZE);
        currentBehavior = pathFollowing;

        // Generate initial path
        generateNewPath();
    }

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
        for (int i = 20; i < 29; i++) {
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
        grid[20][20] = false;
        grid[25][20] = false;
        grid[26][20] = false;
    }

    void setupPatrolPoints() {
        patrolPoints = {
            PatrolPoint(sf::Vector2f(CELL_SIZE * 15, CELL_SIZE * 5)),
            PatrolPoint(sf::Vector2f(CELL_SIZE * 35, CELL_SIZE * 5)),
            PatrolPoint(sf::Vector2f(CELL_SIZE * 15, CELL_SIZE * 15)),
            PatrolPoint(sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 15))
        };
        dtMonster.setupPatrolPoints(patrolPoints);
    }

    void drawEnvironment() {
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

        // Draw patrol points
        for (const auto& point : patrolPoints) {
            sf::CircleShape patrolPoint(5);
            patrolPoint.setFillColor(sf::Color::Yellow);
            patrolPoint.setPosition(point.position.x - 5, point.position.y - 5);
            window.draw(patrolPoint);
        }
    }

    void drawPlayer() {
        // Draw player as a blue circle
        boidSprite.setPosition(character.pos);
float angle = std::atan2(character.vel.y, character.vel.x);
boidSprite.setRotation(angle * 180.0f / M_PI);  // convert to degrees
window.draw(boidSprite);


    }

    bool checkCollision() {
        sf::Vector2f monsterPos = dtMonster.getPosition();
        sf::Vector2f toMonster = monsterPos - character.pos;
        float distance = std::sqrt(toMonster.x * toMonster.x + toMonster.y * toMonster.y);
        return distance < 25.0f;  // Collision radius
    }

    void generateNewPath() {
        // Get current room and next room
        int currentRoom = getCurrentRoom();
        int nextRoom = ((currentRoom % 3) + 1);
        
        // Get room center coordinates
        sf::Vector2f roomCenter;
        switch(nextRoom) {
            case 1: roomCenter = sf::Vector2f(CELL_SIZE * 10, CELL_SIZE * 5); break;
            case 2: roomCenter = sf::Vector2f(CELL_SIZE * 25, CELL_SIZE * 5); break;
            case 3: roomCenter = sf::Vector2f(CELL_SIZE * 20, CELL_SIZE * 15); break;
        }
        
        // Add random offset
        float randX = (rand() % 40 - 20) * 1.5f;
        float randY = (rand() % 40 - 20) * 1.5f;
        sf::Vector2f target = roomCenter + sf::Vector2f(randX, randY);
        
        // Find path
        int startX = static_cast<int>(character.pos.x) / CELL_SIZE;
        int startY = static_cast<int>(character.pos.y) / CELL_SIZE;
        int targetX = static_cast<int>(target.x) / CELL_SIZE;
        int targetY = static_cast<int>(target.y) / CELL_SIZE;
        
        path = findPath(startX, startY, targetX, targetY);
        if (!path.empty()) {
            pathFollowing->setPath(path);
        }
    }

    int getCurrentRoom() {
        float x = character.pos.x;
        float y = character.pos.y;
        
        if (x < CELL_SIZE * 20 && y < CELL_SIZE * 10) return 1;
        if (x >= CELL_SIZE * 20 && y < CELL_SIZE * 10) return 2;
        return 3;
    }

    void run() {
        sf::Clock clock;
        sf::Clock collisionTimer;
        
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed)
                    window.close();
                else if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::R) {
                        // Allow manual respawn with R key
                        respawnPlayer();
                    }
                }
            }

            float deltaTime = clock.restart().asSeconds();

            if (playerCollided) {
                // Wait for 2 seconds after collision before respawning
                if (collisionTimer.getElapsedTime().asSeconds() >= 2.0f) {
                    respawnPlayer();
                    collisionTimer.restart();
                }
            } else {
                // Update behavior timer and switch behaviors
                behaviorTimer += deltaTime;

                if (forcedPathFollow) {
                    if (pathFollowing->isPathComplete() || path.empty()) {
                        generateNewPath();
                    }

                    if (behaviorTimer > 8.0f) {
                        forcedPathFollow = false;
                        currentBehavior = wanderBehaviour;
                        behaviorTimer = 0;
                    }
                } else {
                    if (behaviorTimer > 4.0f) {
                        forcedPathFollow = true;
                        currentBehavior = pathFollowing;
                        behaviorTimer = 0;
                        generateNewPath();
                    }
                }

                // Apply current behavior
                Kinematic target; // Dummy target for wandering
                currentBehavior->apply(character, target, deltaTime);
                update(character, *currentBehavior, deltaTime);

                // Check for new collisions
                if (checkCollision()) {
                    playerCollided = true;
                    collisionTimer.restart();  // Start the respawn timer
                }
            }

            // Update monster
            dtMonster.update(deltaTime, character.pos, grid);

            // Update breadcrumbs
            updateBreadcrumbs(deltaTime);

            // Rendering
            window.clear(sf::Color::White);
            drawEnvironment();
            drawPlayer();
            drawBreadcrumbs();
            dtMonster.draw(window);
            window.display();
        }
    }

    // Add findPath method if not already present
    std::vector<sf::Vector2f> findPath(int startX, int startY, int goalX, int goalY) {
        std::vector<sf::Vector2f> resultPath;
        std::vector<Node*> openSet;
        std::vector<Node*> closedSet;
        
        Node* startNode = new Node(startX, startY);
        Node* goalNode = new Node(goalX, goalY);
        openSet.push_back(startNode);

        const int dx[8] = {-1, 0, 1, 0, -1, -1, 1, 1};
        const int dy[8] = {0, -1, 0, 1, -1, 1, -1, 1};

        while (!openSet.empty()) {
            int currentIndex = 0;
            for (int i = 1; i < openSet.size(); i++) {
                if (openSet[i]->f < openSet[currentIndex]->f) {
                    currentIndex = i;
                }
            }

            Node* current = openSet[currentIndex];

            if (current->x == goalNode->x && current->y == goalNode->y) {
                Node* currentPtr = current;
                while (currentPtr != nullptr) {
                    resultPath.push_back(sf::Vector2f(
                        currentPtr->x * CELL_SIZE + CELL_SIZE / 2,
                        currentPtr->y * CELL_SIZE + CELL_SIZE / 2
                    ));
                    currentPtr = currentPtr->parent;
                }
                std::reverse(resultPath.begin(), resultPath.end());
                
                // Cleanup
                for (auto node : openSet) delete node;
                for (auto node : closedSet) delete node;
                delete goalNode;
                
                return resultPath;
            }

            openSet.erase(openSet.begin() + currentIndex);
            closedSet.push_back(current);

            for (int i = 0; i < 8; i++) {
                int newX = current->x + dx[i];
                int newY = current->y + dy[i];

                if (newX < 0 || newX >= GRID_WIDTH || newY < 0 || newY >= GRID_HEIGHT || grid[newX][newY]) {
                    continue;
                }

                bool inClosedSet = false;
                for (auto node : closedSet) {
                    if (node->x == newX && node->y == newY) {
                        inClosedSet = true;
                        break;
                    }
                }
                if (inClosedSet) continue;

                float tentativeG = current->g + ((i < 4) ? 1.0f : 1.414f);

                Node* neighbor = nullptr;
                bool inOpenSet = false;
                for (auto node : openSet) {
                    if (node->x == newX && node->y == newY) {
                        neighbor = node;
                        inOpenSet = true;
                        break;
                    }
                }

                if (!inOpenSet) {
                    neighbor = new Node(newX, newY);
                    neighbor->g = tentativeG;
                    neighbor->h = std::sqrt(std::pow(goalX - newX, 2) + std::pow(goalY - newY, 2));
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                    openSet.push_back(neighbor);
                }
                else if (tentativeG < neighbor->g) {
                    neighbor->g = tentativeG;
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = current;
                }
            }
        }

        // Cleanup if no path found
        for (auto node : openSet) delete node;
        for (auto node : closedSet) delete node;
        delete goalNode;
        
        return resultPath;
    }
};

int main() {
    try {
      
        // std::cout << "Starting application..." << std::endl;
        // std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
        
        GameWithDTMonster game;
        game.run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}
