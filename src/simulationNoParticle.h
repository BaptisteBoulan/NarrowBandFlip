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

        // Init velocities
        float norm = 2.0f;
        // for (int k = 0; k < size * (size+1); k++) {
        //     grid.new_us[k] = norm * (2.0f * rand()/RAND_MAX - 1.0f);
        //     grid.new_vs[k] = norm * (2.0f * rand()/RAND_MAX - 1.0f);
        // }

        for (int j = size/4; j < 3*size/4; j++) {
            grid.new_us[grid.vIdx(size/2-1, j)] = 1.0f;
            grid.new_us[grid.vIdx(size/2, j)] = 2.0f;
            grid.new_us[grid.vIdx(size/2+1, j)] = 2.0f;
            grid.new_us[grid.vIdx(size/2+2, j)] = 1.0f;
        }
        computeInterpolatedVelocities();
    }

    // MAIN LOOP
    void update(float dt) {
        computeInterpolatedVelocities();
        applyGravity(dt);
        computeDivergences(dt);
        solvePressure(dt);
        applyPressure(dt);
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

    // DEBUG
    void computeR();
    void computeInterpolatedVelocities();
};