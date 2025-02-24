#include <SFML/Graphics.hpp>
#include "Kinematic.h"
#include "SteeringBehaviour.h"
#include "FlockingBehaviour.h"
#include "Update.h"
#include <vector>
#include <deque> 

const int NUM_BOIDS = 100;
const float NEIGHBOR_RADIUS = 90.0f;;

float SteeringBehaviour::maxVelocity = 200.0f;
float SteeringBehaviour::maxAcceleration = 100.0f;
float SteeringBehaviour::maxAngular = 4.0f * M_PI;
float SteeringBehaviour::maxRotation = M_PI;

float SteeringBehaviour::timeToTargetVelocity= 0.2f;
float SteeringBehaviour::timeToTargetRotation=0.3f;
float SteeringBehaviour::radiusOfSatisfaction= 15.0f;
float SteeringBehaviour::radiusOfDeceleration=100.0f;


std::vector<Kinematic> getNeighbors(const Kinematic& boid, const std::vector<Kinematic>& boids) {
    std::vector<Kinematic> neighbors;
    for (const auto& other : boids) {
        if (&boid != &other) {
            sf::Vector2f diff = boid.pos - other.pos;
            float distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            if (distance < NEIGHBOR_RADIUS) {
                neighbors.push_back(other);
            }
        }
    }
    return neighbors;
}

// Utility function for smooth orientation alignment
float alignOrientation(float current, float target, float maxRotation, float dtime) {
    float rotation = target - current;


    while (rotation > M_PI) rotation -= 2.0f * M_PI;
    while (rotation < -M_PI) rotation += 2.0f * M_PI;

    float rotationSize = std::abs(rotation);
    float limitedRotation = std::min(maxRotation * dtime, rotationSize);

    return current + (rotation / rotationSize) * limitedRotation;
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Boids Flocking Simulation");
    sf::Texture boidTexture;
    if (!boidTexture.loadFromFile("boid-sm.png")) {
        return -1;
    }


    std::vector<std::deque<sf::Vector2f>> breadcrumbs(NUM_BOIDS);
    const int maxBreadcrumbs = 20;  

    std::vector<Kinematic> boids(NUM_BOIDS);
    std::vector<sf::Sprite> sprites(NUM_BOIDS);
    std::vector<FlockingBehaviour> behaviours(NUM_BOIDS);

    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids[i].pos = sf::Vector2f(rand() % 800, rand() % 600);
    
        boids[i].orient = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2.0f * M_PI;
    
        boids[i].vel = sf::Vector2f(
            std::cos(boids[i].orient) * SteeringBehaviour::maxVelocity,
            std::sin(boids[i].orient) * SteeringBehaviour::maxVelocity
        );
    
        boids[i].rotation = 0.0f;
    
        sprites[i].setTexture(boidTexture);
        sprites[i].setPosition(boids[i].pos);
        sprites[i].setRotation(boids[i].orient * 180.0f / M_PI);  // Convert radians to degrees
    }

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float dtime = clock.restart().asSeconds();

        for (int i = 0; i < NUM_BOIDS; ++i) {
            std::vector<Kinematic> neighbors = getNeighbors(boids[i], boids);
            behaviours[i].setNeighbors(neighbors);
        
            Kinematic fakeTarget;
            behaviours[i].apply(boids[i], fakeTarget, dtime);
            update(boids[i], behaviours[i], dtime);
        
            // Boundary handling condition is to wrap around the boundary edges
            if (boids[i].pos.x < 0) boids[i].pos.x = 800;
            if (boids[i].pos.x > 800) boids[i].pos.x = 0;
            if (boids[i].pos.y < 0) boids[i].pos.y = 600;
            if (boids[i].pos.y > 600) boids[i].pos.y = 0;
        
            if (boids[i].vel.x != 0 || boids[i].vel.y != 0) {
                float targetOrientation = std::atan2(boids[i].vel.y, boids[i].vel.x);

                boids[i].orient = alignOrientation(boids[i].orient, targetOrientation, SteeringBehaviour::maxAngular, dtime);
            }
        
            sprites[i].setPosition(boids[i].pos);
            sprites[i].setRotation(boids[i].orient * 180.0f / M_PI);  
            breadcrumbs[i].push_back(boids[i].pos);

            if (breadcrumbs[i].size() > maxBreadcrumbs) {
                breadcrumbs[i].pop_front();
            }
        }

        window.clear(sf::Color::White);

        // Render breadcrumbs - For aesthetic purposes i have introduced a fading breadcrumb trail as when i spawn too many sprites on the screen alongwith theor breadcrumbs it creates  ahuge mess and we cant track what is happening
        // had to do for experimentation purposes

        for (const auto& trail : breadcrumbs) {
            int alpha = 50;  
            for (const auto& pos : trail) {
                sf::CircleShape breadcrumb(2.0f);
                breadcrumb.setPosition(pos.x - 1.0f, pos.y - 1.0f);  
                breadcrumb.setFillColor(sf::Color(0, 0, 0, alpha));  
                
                window.draw(breadcrumb);

                alpha += (205 / maxBreadcrumbs);  
            }
        }

        for (auto& sprite : sprites) {
            window.draw(sprite);
        }

        window.display();
    }

    return 0;
}
