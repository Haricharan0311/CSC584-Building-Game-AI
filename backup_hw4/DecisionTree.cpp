#include "DecisionTree.h"
#include <cmath>

float GameState::getCurrentSpeed() const {
    return std::sqrt(character.vel.x * character.vel.x + character.vel.y * character.vel.y);
}

float GameState::getDistanceToNearestWall() const {
    int gridX = static_cast<int>(character.pos.x) / cellSize;
    int gridY = static_cast<int>(character.pos.y) / cellSize;
    
    float minDistance = std::numeric_limits<float>::max();
    
    // Check surrounding cells in a 5x5 area
    for (int dx = -2; dx <= 2; dx++) {
        for (int dy = -2; dy <= 2; dy++) {
            int checkX = gridX + dx;
            int checkY = gridY + dy;
            
            if (checkX >= 0 && checkX < gridWidth && 
                checkY >= 0 && checkY < gridHeight && 
                grid[checkX][checkY]) {
                    
                float dist = std::sqrt(
                    std::pow((checkX * cellSize + cellSize/2) - character.pos.x, 2) +
                    std::pow((checkY * cellSize + cellSize/2) - character.pos.y, 2)
                );
                minDistance = std::min(minDistance, dist);
            }
        }
    }
    
    return minDistance;
}

bool GameState::isInRoom(int roomNumber) const {
    int gridX = static_cast<int>(character.pos.x) / cellSize;
    int gridY = static_cast<int>(character.pos.y) / cellSize;
    
    // Define room boundaries
    switch(roomNumber) {
        case 1: // Top left room
            return gridX < 20 && gridY < 10;
        case 2: // Top right room
            return gridX >= 20 && gridX < 30 && gridY < 10;
        case 3: // Bottom room
            return gridX >= 10 && gridX < 30 && gridY >= 10 && gridY < 20;
        default:
            return false;
    }
}

sf::Vector2f GameState::getRoomCenter(int roomNumber) const {
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

float GameState::getDistanceFromRoomCenter() const {
    sf::Vector2f center;
    if (isInRoom(1)) center = getRoomCenter(1);
    else if (isInRoom(2)) center = getRoomCenter(2);
    else if (isInRoom(3)) center = getRoomCenter(3);
    else return 0;
    
    return std::sqrt(
        std::pow(center.x - character.pos.x, 2) +
        std::pow(center.y - character.pos.y, 2)
    );
}
