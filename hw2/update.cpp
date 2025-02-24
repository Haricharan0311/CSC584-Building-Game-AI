#include "Update.h"

void update(Kinematic& character, SteeringBehaviour& steering, float deltaTime) {
    character.pos += character.vel * deltaTime;
    character.orient += character.rotation * deltaTime;
    character.vel += steering.linear * deltaTime;
    character.rotation += steering.angular * deltaTime;
}