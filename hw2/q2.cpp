#include <SFML/Graphics.hpp>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "Update.h"
#include <vector>

// Initialize static variables
// float SteeringBehaviour::maxVelocity = 400.0f;
// float SteeringBehaviour::maxAcceleration = 300.0f;
// float SteeringBehaviour::maxAngular = 4.0f * M_PI;
// float SteeringBehaviour::maxRotation = 2.0f * M_PI ;

float SteeringBehaviour::maxVelocity = 120.0f;
float SteeringBehaviour::maxAcceleration = 100.0f;
float SteeringBehaviour::maxAngular = 1.0f * M_PI;
float SteeringBehaviour::maxRotation = M_PI / 2.0f ;

float SteeringBehaviour::timeToTargetVelocity= 0.2f;
float SteeringBehaviour::timeToTargetRotation=0.3f;
float SteeringBehaviour::radiusOfSatisfaction= 15.0f;
float SteeringBehaviour::radiusOfDeceleration=100.0f;
// float timeToTargetVelocity = .5f;         
// float timeToTargetRotation = 0.5f;          
// float radiusOfSatisfaction = 20.0f;         
// float radiusOfDeceleration = 150.0f; 

class Crumb : public sf::CircleShape {
public:
    Crumb() {
        this->setRadius(5.f);
        this->setFillColor(sf::Color(0, 0, 255, 255));
        this->setPosition(-100, -100); // Initialize off-screen
    }

    void drop(sf::Vector2f position) {
        this->setPosition(position);
    }
};

class CombinedBehaviour : public SteeringBehaviour {
private:
    PositionBehaviour positionBehaviour;
    OrientationBehaviour orientationBehaviour;

public:
    void apply(Kinematic& kinematic, const Kinematic& target, float dtime) override {
        positionBehaviour.apply(kinematic, target, dtime);
        orientationBehaviour.apply(kinematic, target, dtime);
        linear = positionBehaviour.linear;
        angular = orientationBehaviour.angular;
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Mouse Control with Breadcrumbs - Slow and Gradual Movement");
    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-sm.png")) {
        return -1;
    }

    sf::Sprite boidSprite(boidTexture);
    Kinematic character;
    character.pos = sf::Vector2f(400.0f, 300.0f);
    character.vel = sf::Vector2f(0.0f, 0.0f);
    character.orient = 0.0f;
    character.rotation = 0.0f;

    Kinematic target;
    target.pos = character.pos;
    target.orient = character.orient;

    CombinedBehaviour steering;
    sf::Clock clock;

    std::vector<Crumb> breadcrumbs;
    float breadcrumbTimer = 0.1f;
    const float breadcrumbDropRate = 0.1f;
    const size_t maxBreadcrumbs = 20; // Maximum number of breadcrumbs

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    target.pos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
                }
            }
        }

        float dtime = clock.restart().asSeconds();
        steering.apply(character, target, dtime);
        update(character, steering, dtime);

        // Update orientation
        if (character.vel.x != 0 || character.vel.y != 0) {
            character.orient = std::atan2(character.vel.y, character.vel.x);
        }

        breadcrumbTimer -= dtime;
        if (breadcrumbTimer <= 0) {
            breadcrumbTimer = breadcrumbDropRate;
            Crumb newCrumb;
            newCrumb.drop(character.pos);
            breadcrumbs.push_back(newCrumb);

            // Keep the breadcrumbs count limited
            if (breadcrumbs.size() > maxBreadcrumbs) {
                breadcrumbs.erase(breadcrumbs.begin());
            }
        }


        boidSprite.setPosition(character.pos);
        boidSprite.setRotation(character.orient * 180.0f / M_PI);

        window.clear(sf::Color::White);
        for (auto& crumb : breadcrumbs) {
            window.draw(crumb);
        }
        window.draw(boidSprite);
        window.display();
    }
    return 0;
}
