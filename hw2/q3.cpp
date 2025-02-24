#include <SFML/Graphics.hpp>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "Update.h"

float SteeringBehaviour::maxVelocity = 90.0f;
float SteeringBehaviour::maxAcceleration = 10.0f;
float SteeringBehaviour::maxAngular = 2.0f * M_PI;
float SteeringBehaviour::maxRotation = M_PI;

float SteeringBehaviour::timeToTargetVelocity = 0.2f;
float SteeringBehaviour::timeToTargetRotation = 0.3f;
float SteeringBehaviour::radiusOfSatisfaction = 15.0f;
float SteeringBehaviour::radiusOfDeceleration = 100.0f;

const float WANDER_RADIUS = 50.0f;
const float WANDER_OFFSET = 100.0f;
const float WANDER_RATE = 0.5f;
const int MAX_BREADCRUMBS = 30;
const float BREADCRUMB_DELAY = 0.2f; 

class RandomWanderBehaviour : public SteeringBehaviour {
private:
    float wanderOrientation;

public:
    RandomWanderBehaviour() {
        wanderOrientation = 0.0f;
        std::srand(std::time(0));
    }

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        wanderOrientation += ((static_cast<float>(std::rand()) / RAND_MAX) - 0.5f) * WANDER_RATE;
        float targetOrientation = kinematic.orient + wanderOrientation;
        
        sf::Vector2f wanderCircleCenter = kinematic.pos + WANDER_OFFSET * sf::Vector2f(std::cos(kinematic.orient), std::sin(kinematic.orient));
        sf::Vector2f displacement = WANDER_RADIUS * sf::Vector2f(std::cos(targetOrientation), std::sin(targetOrientation));
        sf::Vector2f targetPos = wanderCircleCenter + displacement;

        Kinematic newTarget;
        newTarget.pos = targetPos;
        newTarget.orient = targetOrientation;
        
        PositionBehaviour positionBehaviour;
        OrientationBehaviour orientationBehaviour;
        positionBehaviour.apply(kinematic, newTarget, dtime);
        orientationBehaviour.apply(kinematic, newTarget, dtime);
        
        linear = positionBehaviour.linear;
        angular = orientationBehaviour.angular;
    }
};

class SmoothWanderBehaviour : public SteeringBehaviour {
private:
    float wanderOrientation;

public:
    SmoothWanderBehaviour() {
        wanderOrientation = 0.0f;
        std::srand(std::time(0));
    }

    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        wanderOrientation += ((static_cast<float>(std::rand()) / RAND_MAX) - 0.5f) * WANDER_RATE;
        wanderOrientation *= 0.9f; // Smoothing factor
        float targetOrientation = kinematic.orient + wanderOrientation;
        
        sf::Vector2f wanderCircleCenter = kinematic.pos + WANDER_OFFSET * sf::Vector2f(std::cos(kinematic.orient), std::sin(kinematic.orient));
        sf::Vector2f displacement = WANDER_RADIUS * sf::Vector2f(std::cos(targetOrientation), std::sin(targetOrientation));
        sf::Vector2f targetPos = wanderCircleCenter + displacement;

        Kinematic newTarget;
        newTarget.pos = targetPos;
        newTarget.orient = targetOrientation;
        
        PositionBehaviour positionBehaviour;
        OrientationBehaviour orientationBehaviour;
        positionBehaviour.apply(kinematic, newTarget, dtime);
        orientationBehaviour.apply(kinematic, newTarget, dtime);
        
        linear = positionBehaviour.linear;
        angular = orientationBehaviour.angular;
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Wander Demo");
    

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        return -1;
    }

    // Create Text Box
    sf::Text instructionText;
    instructionText.setFont(font);
    instructionText.setString("Press SPACE to toggle\nbetween Random and Smooth Wander");
    instructionText.setCharacterSize(20);
    instructionText.setFillColor(sf::Color::Black);
    instructionText.setPosition(10, 10);

    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-sm.png")) {
        return -1;
    }

    sf::Sprite boidSprite(boidTexture);
    Kinematic character;
    character.pos = sf::Vector2f(320.0f, 240.0f);
    character.vel = sf::Vector2f(0.0f, 0.0f);
    character.orient = 0.0f;
    character.rotation = 0.0f;

    RandomWanderBehaviour randomWander;
    SmoothWanderBehaviour smoothWander;
    SteeringBehaviour* currentWander;
    std::queue<sf::Vector2f> breadcrumbs;
    sf::Clock clock;
    sf::Clock breadcrumbClock;
    bool useSmooth = false;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
                useSmooth = !useSmooth;
            }
        }
    
        float dtime = clock.restart().asSeconds();
        currentWander = useSmooth ? (SteeringBehaviour*)&smoothWander : (SteeringBehaviour*)&randomWander;
        currentWander->apply(character, character, dtime);
        update(character, *currentWander, dtime);
    

        if (character.vel.x != 0 || character.vel.y != 0) {
            character.orient = std::atan2(character.vel.y, character.vel.x);
        }
    
        boidSprite.setPosition(character.pos);
        boidSprite.setRotation(character.orient * 180.0f / M_PI);
    
// Screen wrapping is the way boundary conditions are handled in this wander 
        if (character.pos.x < 0) character.pos.x = window.getSize().x;
        if (character.pos.x > window.getSize().x) character.pos.x = 0;
        if (character.pos.y < 0) character.pos.y = window.getSize().y;
        if (character.pos.y > window.getSize().y) character.pos.y = 0;
    
        // Breadcrumb trail logic
        if (breadcrumbClock.getElapsedTime().asSeconds() > 0.2f) {
            breadcrumbs.push(character.pos);
            if (breadcrumbs.size() > 30) {
                breadcrumbs.pop();
            }
            breadcrumbClock.restart();
        }
    
        // Render everything
        window.clear(sf::Color::White);
        window.draw(boidSprite);
    
        std::queue<sf::Vector2f> tempBreadcrumbs = breadcrumbs;
        while (!tempBreadcrumbs.empty()) {
            sf::Vector2f crumb = tempBreadcrumbs.front();
            tempBreadcrumbs.pop();
    
            sf::CircleShape breadcrumb(3);
            breadcrumb.setFillColor(sf::Color::Red);
            breadcrumb.setPosition(crumb);
            window.draw(breadcrumb);
        }
    
        // Draw the instruction text
        window.draw(instructionText);
    
        window.display();
    }

    return 0;
}