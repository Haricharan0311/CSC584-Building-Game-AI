#include "Kinematics.h"

// Implementation of the update function
void Kinematic::update(Steering& steering, float dtime) {
        pos += vel * dtime;                   // Update position
        orient += rotation * dtime;          // Update orientation
        vel += steering.linear * dtime;      // Update velocity
        rotation += steering.angular * dtime; // Update angular velocity
    }
