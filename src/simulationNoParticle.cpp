#include "simulationNoParticle.h"


// === MAIN STEPS ===

void Simulation::applyGravity(float dt) {
    for (int k = 0; k < (size+1)*size; k++) {
        grid.new_vs[k] += GRAVITY * dt;
    }
}

void Simulation::computeDivergences(float dt) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) {
                // If a face borders a solid cell, the velocity is forced to 0
                grid.new_us[grid.uIdx(i + 1, j)] = 0.0f;
                grid.new_us[grid.uIdx(i, j)] = 0.0f;
                grid.new_vs[grid.vIdx(i, j + 1)] = 0.0f;
                grid.new_vs[grid.vIdx(i, j)] = 0.0f;
            }
        }
    }
    
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) {
                grid.divergence[grid.gridIdx(i, j)] = 0;
                continue;
            }

            float u_right = grid.new_us[grid.uIdx(i + 1, j)];
            float u_left  = grid.new_us[grid.uIdx(i, j)];
            float v_top   = grid.new_vs[grid.vIdx(i, j + 1)];
            float v_bot   = grid.new_vs[grid.vIdx(i, j)];

            float div = (u_right - u_left) + (v_top - v_bot);
            grid.divergence[grid.gridIdx(i, j)] = (div / h) * (RHO / dt);
        }
    }
}

void Simulation::computePressures(float dt) {
    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) continue; // Skip solid cells

            float sumPressures = 0.0f;
            int fluidNeighbors = 0;

            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};
            
            for (auto& n : neighbors) {
                int ni = n[0];
                int nj = n[1];
                if (!grid.solidCells[grid.gridIdx(ni, nj)]) {
                    sumPressures += grid.pressure[grid.gridIdx(ni, nj)];
                    fluidNeighbors++;
                }
            }

            if (fluidNeighbors > 0) {
                int idx = grid.gridIdx(i, j);
                grid.pressure[idx] = (sumPressures - (h * h * grid.divergence[idx])) / fluidNeighbors;
            }
        }
    }
}

void Simulation::solvePressure(float dt) {
    for (int k = 0; k < grid.total_size; k++) grid.pressure[k] = 0;

    // Pressure solving iterations
    for (int k = 0; k < 100; k++) {
        computePressures(dt);
    }
    computeR();

    
}

void Simulation::applyPressure(float dt) {
    float K = dt / RHO / h;

    // U VELOCITIES
    for (int i = 1; i < size; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (!grid.solidCells[grid.gridIdx(i, j)] && !grid.solidCells[grid.gridIdx(i - 1, j)]) {
                grid.new_us[grid.uIdx(i, j)] -= K * (grid.pressure[grid.gridIdx(i, j)] - grid.pressure[grid.gridIdx(i - 1, j)]);
            } else {
                grid.new_us[grid.uIdx(i, j)] = 0.0f; // Solid boundary
            }
        }
    }

    // V VELOCITIES
    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size; j++) {
            if (!grid.solidCells[grid.gridIdx(i, j)] && !grid.solidCells[grid.gridIdx(i, j-1)]) {
                grid.new_vs[grid.vIdx(i, j)] -= K * (grid.pressure[grid.gridIdx(i, j)] - grid.pressure[grid.gridIdx(i, j-1)]);
            } else {
                grid.new_vs[grid.vIdx(i, j)] = 0.0f; // Solid boundary
            }
        }
    }

    computeDivergences(dt); // just to debug
}

void Simulation::computeR() {
    double sumSquaredResiduals = 0.0;
    int activeCells = 0;

    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) continue;

            float p_ij = grid.pressure[grid.gridIdx(i, j)];
            float sumPNeighbors = 0.0f;
            int fluidNeighbors = 0;

            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};
            for (auto& n : neighbors) {
                if (!grid.solidCells[grid.gridIdx(n[0], n[1])]) {
                    sumPNeighbors += grid.pressure[grid.gridIdx(n[0], n[1])];
                    fluidNeighbors++;
                }
            }

            if (fluidNeighbors > 0) {
                float targetP = (sumPNeighbors - (h * h * grid.divergence[grid.gridIdx(i, j)])) / fluidNeighbors;
                float localResidual = targetP - p_ij; 
                
                sumSquaredResiduals += localResidual * localResidual;
                activeCells++;
            }
        }
    }
    double residualNorm = std::sqrt(sumSquaredResiduals / (activeCells > 0 ? activeCells : 1));
    std::cout << "Residual Norm (Jacobi Delta): " << residualNorm << std::endl;
}

void Simulation::computeInterpolatedVelocities() {
        // DEBUG
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                grid.interpolatedVelocities[grid.gridIdx(i,j)].x = 0.5f * (grid.new_us[grid.uIdx(i,j)]+grid.new_us[grid.uIdx(i+1,j)]);
                grid.interpolatedVelocities[grid.gridIdx(i,j)].y = 0.5f * (grid.new_vs[grid.vIdx(i,j)]+grid.new_vs[grid.vIdx(i,j+1)]);
            }
        }
}