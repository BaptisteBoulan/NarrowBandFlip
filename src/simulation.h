#pragma once
#include "Grid.h"
#include "Particle.h"

class Simulation {
public:
    int size;
    float h;
    Grid grid;
    std::vector<Particle> particles;

    // CONSTRUCTOR
    Simulation(int size) : size(size), grid(size), h(1.0f/size) {
        for(float x = 0.5f; x < 0.9f; x += 0.005f) {
            for (float y = 0.4f; y < 0.9f; y += 0.005f) {
                particles.emplace_back(glm::vec2(x,y));
            }
        }
    }

    // MAIN LOOP
    void update(float dt) {
        p2g();
        applyGravity(dt);
        computeDivergences(dt);
        solvePressure(dt);
        applyPressure(dt);
        
        g2p(dt);
        advectParticles(dt);
    }

private:
    float RHO = 1.0f;
    float GRAVITY = -9.81f;

    // STEPS
    void p2g();
    void applyGravity(float dt);
    void computeDivergences(float dt);
    void computePressures(float dt);
    void solvePressure(float dt);
    void applyPressure(float dt);
    void g2p(float dt);
    void advectParticles(float dt);

    // HELPERS
    float interpolate(float x, float y, const std::vector<float>& gridValues, bool isU);
    glm::vec2 sampleVelocityFromGrid(glm::vec2 pos);
};