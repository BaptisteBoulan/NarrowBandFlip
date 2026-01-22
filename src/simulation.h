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
        for(float x = 0.2f; x < 0.8f; x += 0.01f) {
            for (float y = 0.2f; y < 0.8f; y += 0.01f) {

                float theta = (abs(x-0.5f)<1e-3) ? (float)M_PI/2.0f : (float)atan((y-0.5f)/(x-0.5f));
                if (x<0.5f) theta += (float)M_PI;

                particles.emplace_back(glm::vec2(x,y), 2.0f * glm::vec2(-sin(theta), cos(theta)));

                // std::cout<<x<<" "<<y<<" "<<theta<<std::endl;
            }
        }
    }
    
    // STEPS
    void p2g();
    void applyGravity(float dt);
    void computeDivergences(float dt);
    void solvePressure(float dt);
    void applyPressure(float dt);
    void g2p(float dt);

private:
    float RHO = 1.0f;
    float GRAVITY = -9.81f;
    // float GRAVITY = 0;


    // HELPERS
    void applyA(const std::vector<float>& x, std::vector<float>& Ax);
    float dotProduct(const std::vector<float>& a, const std::vector<float>& b);
};