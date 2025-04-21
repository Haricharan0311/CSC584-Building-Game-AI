#ifndef GAME_STATE_H
#define GAME_STATE_H

#include <SFML/System/Vector2.hpp>
#include <memory>
#include "SteeringBehaviour.h"
#include "Kinematic.h"

class GameState {
private:
    const Kinematic& character;
    const bool (*grid)[30];
    const int gridWidth;
    const int gridHeight;
    const int cellSize;
    mutable std::shared_ptr<SteeringBehaviour> currentBehavior;
    float timeInState;

public:
    GameState(const Kinematic& c, const bool (*g)[30], int width, int height, int cs)
        : character(c), grid(g), gridWidth(width), gridHeight(height), 
          cellSize(cs), timeInState(0) {}

    void setBehavior(std::shared_ptr<SteeringBehaviour> behavior) const {
        currentBehavior = behavior;
    }

    std::shared_ptr<SteeringBehaviour> getCurrentBehavior() const {
        return currentBehavior;
    }

    float getCurrentSpeed() const {
        return std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
    }

    float getDistanceToNearestWall() const {
        float minDist = 1000.0f;
        int currentX = static_cast<int>(character.pos.x) / cellSize;
        int currentY = static_cast<int>(character.pos.y) / cellSize;

        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                int checkX = currentX + dx;
                int checkY = currentY + dy;
                
                if (checkX >= 0 && checkX < gridWidth && 
                    checkY >= 0 && checkY < gridHeight && 
                    grid[checkX][checkY]) {
                    sf::Vector2f wallCenter(
                        checkX * cellSize + cellSize/2.0f,
                        checkY * cellSize + cellSize/2.0f
                    );
                    sf::Vector2f toWall = character.pos - wallCenter;
                    float dist = std::sqrt(toWall.x * toWall.x + toWall.y * toWall.y);
                    minDist = std::min(minDist, dist);
                }
            }
        }
        return minDist;
    }

    bool isInRoom(int roomNumber) const {
        int x = static_cast<int>(character.pos.x) / cellSize;
        int y = static_cast<int>(character.pos.y) / cellSize;
        
        switch(roomNumber) {
            case 1: // Top left room
                return x < 20 && y < 10;
            case 2: // Top right room
                return x >= 20 && x < 30 && y < 10;
            case 3: // Bottom room
                return x >= 10 && x < 30 && y >= 10 && y < 20;
            default:
                return false;
        }
    }

    sf::Vector2f getRoomCenter(int roomNumber) const {
        switch(roomNumber) {
            case 1: // Top left room
                return sf::Vector2f(10 * cellSize, 5 * cellSize);
            case 2: // Top right room
                return sf::Vector2f(25 * cellSize, 5 * cellSize);
            case 3: // Bottom room
                return sf::Vector2f(20 * cellSize, 15 * cellSize);
            default:
                return sf::Vector2f(0, 0);
        }
    }

    void updateStateTime(float deltaTime) {
        timeInState += deltaTime;
    }

    float getTimeInCurrentState() const {
        return timeInState;
    }

    void resetStateTime() {
        timeInState = 0;
    }
};

#endif // GAME_STATE_H
