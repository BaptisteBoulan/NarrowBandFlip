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
        // Init walls
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                    grid.solidCells[grid.gridIdx(i, j)] = true;
                } else {
                    grid.solidCells[grid.gridIdx(i, j)] = false;
                }
            }
        }

        // Init particles
        for(float x = 0.6f; x < 0.9f; x += 0.03f) {
            for (float y = 0.05f; y < 0.9f; y += 0.03f) {
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

    // HELPERS
    void applyA(const std::vector<float>& x, std::vector<float>& Ax);
    float dotProduct(const std::vector<float>& a, const std::vector<float>& b);
};