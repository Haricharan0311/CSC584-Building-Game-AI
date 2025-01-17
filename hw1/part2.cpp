#include <SFML/Graphics.hpp>
#include <iostream>
#include "Kinematics.h"

int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Part 2");

    //Using boid-sm
    sf::Texture texture;
    if (!texture.loadFromFile("boid-sm.png")) {
        std::cerr << "Error loading texture from file\n";
        return -1;
    }

    sf::Sprite sprite(texture);
    sprite.setPosition(0.f, 20.f); // Top-left corner with an offset to make it fully visible

    Kinematic kinematic;
    kinematic.pos = sf::Vector2f(0.f, 20.f);   // Start at the top-left corner
    kinematic.vel = sf::Vector2f(100.f, 0.f); // Constant speed of 100 pixels/second to the right
    kinematic.orient = 0.f;                   
    kinematic.rotation = 0.f;                 

    // No acceleration
    Steering steering;
    steering.linear = sf::Vector2f(0.f, 0.f); 
    steering.angular = 0.f;                  

    // Clock for delta time
    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        float deltaTime = clock.restart().asSeconds();

        kinematic.update(steering, deltaTime);

        if (kinematic.pos.x > 640.f) {
            kinematic.pos.x = 0.f; 
        }

        sprite.setPosition(kinematic.pos);

        window.clear(sf::Color::White);
        window.draw(sprite);
        window.display();
    }

    return 0;
}
