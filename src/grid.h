#pragma once
#include "config.h"

enum class CellType {
    SOLID,
    AIR,
    FLUID
};

class Grid {
public:
    int size;
    int total_cells;

    // Cell-centered data
    std::vector<float> pressure;
    std::vector<float> divergence;
    std::vector<CellType> cellType;

    // Level set, cell-centered too
    std::vector<float> levelSet;
    std::vector<float> newLevelSet;
    std::vector<float> particlesLevelSet;
    std::vector<float> finalLevelSet;

    // Face-centered velocities (Staggered)
    std::vector<float> us, new_us, uMasses;
    std::vector<float> vs, new_vs, vMasses;
    std::vector<float> ws, new_ws, wMasses;

    Grid(int size) : size(size) {
        total_cells = size * size * size;

        pressure.resize(total_cells, 0.0f);
        divergence.resize(total_cells, 0.0f);
        cellType.resize(total_cells, CellType::AIR);
        
        levelSet.resize(total_cells, 0.0f);
        newLevelSet.resize(total_cells, 0.0f);
        particlesLevelSet.resize(total_cells, 0.0f);
        finalLevelSet.resize(total_cells, 0.0f);

        int uCount = (size + 1) * size * size;
        int vCount = size * (size + 1) * size;
        int wCount = size * size * (size + 1);

        us.resize(uCount, 0.0f); new_us.resize(uCount, 0.0f); uMasses.resize(uCount, 0.0f);
        vs.resize(vCount, 0.0f); new_vs.resize(vCount, 0.0f); vMasses.resize(vCount, 0.0f);
        ws.resize(wCount, 0.0f); new_ws.resize(wCount, 0.0f); wMasses.resize(wCount, 0.0f);
    }

    int gridIdx(int x, int y, int z = 0) const { 
        return (z * size * size) + (y * size) + x; 
    }

    int uIdx(int x, int y, int z = 0) const { 
        return (z * size * (size + 1)) + (y * (size + 1)) + x; 
    }

    int vIdx(int x, int y, int z = 0) const { 
        return (z * (size + 1) * size) + (y * size) + x; 
    }

    int wIdx(int x, int y, int z = 0) const { 
        return (z * size * size) + (y * size) + x; 
    }
};