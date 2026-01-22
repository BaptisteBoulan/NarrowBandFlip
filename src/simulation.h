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
                    grid.cellType[grid.gridIdx(i, j)] = CellType::SOLID;
                } else {
                    grid.cellType[grid.gridIdx(i, j)] = CellType::AIR;
                }
            }
        }

        for (int i = size / 2 - 2; i <= size / 2 + 2; i++) {
            for (int j = size / 4 - 2; j <= size / 4 + 2; j++) {
                grid.cellType[grid.gridIdx(i, j)] = CellType::SOLID;
            }
        }

        // Init particles
        glm::vec2 p1(0.2f, 0.4f);
        glm::vec2 p2(0.8f, 0.8f);
        float spacing = 0.02f;

        for(float x = p1.x; x < p2.x; x += spacing) {
            for (float y = p1.y; y < p2.y; y += spacing) {

                particles.emplace_back(glm::vec2(x,y));
            }
        }
    }

    // STEPS
    void p2g();
    void applyForces(float dt);
    void computeDivergences(float dt);
    void solvePressure(float dt);
    void applyPressure(float dt);
    void g2p(float dt);

    // HELPERS
    void addParticle(glm::vec2 pos);

private:
    float RHO = 1.0f;
    float GRAVITY = -9.81f;

    // HELPERS
    void applyA(const std::vector<float>& x, std::vector<float>& Ax);
    float dotProduct(const std::vector<float>& a, const std::vector<float>& b);
    void classifyCells();
};