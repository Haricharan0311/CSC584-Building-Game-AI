#include <SFML/Graphics.hpp>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "Update.h"


float SteeringBehaviour::timeToTargetVelocity = 0.2f;
float SteeringBehaviour::timeToTargetRotation = 0.3f;
float SteeringBehaviour::radiusOfSatisfaction = 15.0f;
float SteeringBehaviour::radiusOfDeceleration = 100.0f;

int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Velocity Matching");
    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-sm.png")) {
        return -1;
    }

    sf::Sprite boidSprite(boidTexture);
    sf::Clock clock;
    sf::Vector2f lastMousePos;
    bool firstUpdate = true;
    int updateCounter = 0;
    const int breadcrumbUpdateRate = 10; // Update breadcrumbs every 10 position updates
    
    Kinematic character;
    // I am trying ot get the boid to get to the screen center alongwith the mouse and then start the velocity matching as it makes more sense then
    character.pos = sf::Vector2f(320.0f, 240.0f); // Default to screen center initially
    character.vel = sf::Vector2f(0.0f, 0.0f);
    character.orient = 0.0f;
    character.rotation = 0.0f;

    Kinematic target;
    target.vel = sf::Vector2f(0.0f, 0.0f);

    VelocityBehaviour velocityMatching;
    std::vector<sf::CircleShape> breadcrumbs;
    const size_t maxBreadcrumbs = 40; 

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dtime = clock.restart().asSeconds();
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        sf::Vector2f worldMousePos = window.mapPixelToCoords(mousePos);

        if (firstUpdate) {
            character.pos = worldMousePos; // Start at the initial mouse position - this is where i tru getting the boid to start with where the mouse is. If not the boid gets spawned randomly somewhere
            lastMousePos = worldMousePos;  
            firstUpdate = false;
        }

        target.vel = (worldMousePos - lastMousePos) / dtime;
        lastMousePos = worldMousePos;
        
        velocityMatching.apply(character, target, dtime);
        update(character, velocityMatching, dtime);

        if (character.vel.x != 0 || character.vel.y != 0) {
            character.orient = std::atan2(character.vel.y, character.vel.x);
        }

        boidSprite.setPosition(character.pos);
        boidSprite.setRotation(character.orient * 180.0f / M_PI);

        // Update breadcrumbs only every few updates
        if (updateCounter % breadcrumbUpdateRate == 0) {
            if (breadcrumbs.size() > maxBreadcrumbs) {
                breadcrumbs.erase(breadcrumbs.begin());
            }
            sf::CircleShape breadcrumb(3);
            breadcrumb.setPosition(character.pos);
            breadcrumb.setFillColor(sf::Color::Black);
            breadcrumbs.push_back(breadcrumb);
        }
        updateCounter++;

        window.clear(sf::Color::White);
        for (const auto& crumb : breadcrumbs) {
            window.draw(crumb);
        }
        window.draw(boidSprite);
        window.display();
    }

    return 0;
}
