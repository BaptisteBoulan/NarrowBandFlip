#pragma once
#include "config.h"

struct Particle {
    glm::vec2 pos;
    glm::vec2 vel;

    Particle(glm::vec2 pos) : pos(pos), vel(0.0f) {}
    Particle(glm::vec2 pos,glm::vec2 vel) : pos(pos), vel(vel) {}
};