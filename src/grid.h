#pragma once
#include "config.h"

class Grid {
public:
    int size, total_size;
    std::vector<float> pressure;
    std::vector<float> divergence;

    std::vector<float> us; // horizontal velocities
    std::vector<float> vs; // vertical velocities

    std::vector<float> new_us; // horizontal velocities
    std::vector<float> new_vs; // vertical velocities

    std::vector<float> uMasses; // horizontal masses
    std::vector<float> vMasses; // vertical masses


    Grid(int size) : size(size) {
        total_size = size * size;

        pressure.resize(total_size, 0.0f);
        divergence.resize(total_size, 0.0f);
        us.resize(total_size + size, 0.0f);
        vs.resize(total_size + size, 0.0f);
        new_us.resize(total_size + size, 0.0f);
        new_vs.resize(total_size + size, 0.0f);
        uMasses.resize(total_size + size, 0.0f);
        vMasses.resize(total_size + size, 0.0f);
    }
    
    int gridIdx(int x, int y) const { return y * size + x; }
    int uIdx(int x, int y) const { return y * (size + 1) + x; }
    int vIdx(int x, int y) const { return y * size + x; }
};