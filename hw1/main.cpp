#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include "Kinematics.h"

struct SpriteData {
        Kinematic kinematic;
        sf::Sprite sprite;
    };
using namespace std;
int main() {
    sf::RenderWindow window(sf::VideoMode(640, 480), "Part 3");

    sf::Texture texture;
    if (!texture.loadFromFile("boid-sm.png")) {
        std::cerr << "Error loading texture from file\n";
        return -1;
    }

    float offsetX = 20.f;
    float offsetY = 20.f;

    float windowWidth = 640.f;
    float windowHeight = 480.f;
    float regionWidth = windowWidth - 2 * offsetX;
    float regionHeight = windowHeight - 2 * offsetY;
    //Subtract offsetX twice from the total width (once for the left margin and once for the right margin).
    //Similarly, subtract offsetY twice from the total height.

    // Steering data - No Acceleration
    Steering steering;
    steering.linear = sf::Vector2f(0.f, 0.f); // Default steering
    steering.angular = 0.f;

    vector<SpriteData> sprites;

    sf::Clock clock;
    float cycleDuration = 2.0f; // Timing logic is to make sure there is a unform time of 2 from end to end
    float speedX = regionWidth / cycleDuration;
    float speedY = regionHeight / cycleDuration;

    // Initialize the first sprite
    SpriteData firstSprite;
    firstSprite.kinematic.pos = sf::Vector2f(offsetX, offsetY); // Top-left corner
    firstSprite.kinematic.vel = sf::Vector2f(speedX, 0.f);     // Move right
    firstSprite.kinematic.orient = 0.f;
    firstSprite.kinematic.rotation = 0.f;
    firstSprite.sprite.setTexture(texture);
    firstSprite.sprite.setPosition(firstSprite.kinematic.pos);
    sprites.push_back(firstSprite);

    while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    // Calculate delta time
    float deltaTime = clock.restart().asSeconds();

    for (auto& spriteData : sprites) {
        spriteData.kinematic.update(steering, deltaTime);
        spriteData.sprite.setPosition(spriteData.kinematic.pos);
        spriteData.sprite.setRotation(spriteData.kinematic.orient * 180.f / 3.14159265359f);
    }

    for (auto& spriteData : sprites) {
        if (spriteData.kinematic.vel.x > 0 && spriteData.kinematic.pos.x >= windowWidth - offsetX) {
            spriteData.kinematic.pos.x = windowWidth - offsetX;
            spriteData.kinematic.vel = sf::Vector2f(0.f, speedY); 
            spriteData.kinematic.orient = 3.14159265359f / 2;     
            if (sprites.size() < 4) {
                SpriteData newSprite;
                newSprite.kinematic.pos = sf::Vector2f(offsetX, offsetY);
                newSprite.kinematic.vel = sf::Vector2f(speedX, 0.f);
                newSprite.kinematic.orient = 0.f;
                newSprite.sprite.setTexture(texture);
                newSprite.sprite.setPosition(newSprite.kinematic.pos);
                sprites.push_back(newSprite);
            }
        } else if (spriteData.kinematic.vel.y > 0 && spriteData.kinematic.pos.y >= windowHeight - offsetY) {
            spriteData.kinematic.pos.y = windowHeight - offsetY;
            spriteData.kinematic.vel = sf::Vector2f(-speedX, 0.f); 
            spriteData.kinematic.orient = 3.14159265359f;          
        } else if (spriteData.kinematic.vel.x < 0 && spriteData.kinematic.pos.x <= offsetX) {
            spriteData.kinematic.pos.x = offsetX;
            spriteData.kinematic.vel = sf::Vector2f(0.f, -speedY); 
            spriteData.kinematic.orient = 3.14159265359f * 1.5f;  
        }
    }

    // Lambda fn to remove when top left hit 
    sprites.erase(
        std::remove_if(sprites.begin(), sprites.end(), [&](const SpriteData& spriteData) {
            return (spriteData.kinematic.vel.y < 0 && spriteData.kinematic.pos.y <= offsetY);
        }),
        sprites.end()
    );

    if (sprites.empty()) {
        SpriteData newSprite;
        newSprite.kinematic.pos = sf::Vector2f(offsetX, offsetY);
        newSprite.kinematic.vel = sf::Vector2f(speedX, 0.f);
        newSprite.kinematic.orient = 0.f;
        newSprite.sprite.setTexture(texture);
        newSprite.sprite.setPosition(newSprite.kinematic.pos);
        sprites.push_back(newSprite);
    }

    window.clear(sf::Color::White);
    for (const auto& spriteData : sprites) {
        window.draw(spriteData.sprite);
    }
    window.display();
}


    return 0;
}
