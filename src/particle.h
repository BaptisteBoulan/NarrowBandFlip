#pragma once
#include "config.h"

struct Particle {
    glm::vec2 pos;
    glm::vec2 vel;
    glm::vec3 color;

    Particle(glm::vec2 pos) : pos(pos), vel(0.0f), color(1.0f) {}
};