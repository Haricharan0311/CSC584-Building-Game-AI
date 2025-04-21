#ifndef MONSTER_INTERFACE_H
#define MONSTER_INTERFACE_H

#include <SFML/System/Vector2.hpp>

// Interface for Monster class
class MonsterInterface {
public:
    virtual ~MonsterInterface() = default;
    virtual const sf::Vector2f& getPosition() const = 0;
    virtual void setPosition(const sf::Vector2f& p) = 0;
    virtual void setVelocity(const sf::Vector2f& v) = 0;
    virtual float getRotation() const = 0;
    virtual void setRotation(float r) = 0;
    virtual void setScale(float s) = 0;
    virtual void resetScale() = 0;
};

#endif // MONSTER_INTERFACE_H
