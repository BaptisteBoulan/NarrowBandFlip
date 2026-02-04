#pragma once
#include "config.h"

struct Particle {
    glm::vec4 pos;
    glm::vec4 vel;

    Particle(glm::vec2 pos) : pos(pos, 0.0f, 0.0f), vel(0.0f) {}
    Particle(glm::vec3 pos) : pos(pos, 0.0f), vel(0.0f) {}
};