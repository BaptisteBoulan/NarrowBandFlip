#include "simulation.h"


// === MAIN STEPS ===

void Simulation::p2g() {

    // DEBUG
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            grid.interpolatedVelocities[grid.gridIdx(i,j)].x = 0.5f * (grid.new_us[grid.uIdx(i,j)]+grid.new_us[grid.uIdx(i+1,j)]);
            grid.interpolatedVelocities[grid.gridIdx(i,j)].y = 0.5f * (grid.new_vs[grid.vIdx(i,j)]+grid.new_vs[grid.vIdx(i,j+1)]);
        }
    }

    // RESET GRIDS
    grid.us.assign(size * (size + 1), 0.0f);
    grid.vs.assign((size + 1) * size, 0.0f); 

    grid.uMasses.assign(size * (size + 1), 0.0f);
    grid.vMasses.assign((size + 1) * size, 0.0f);

    for (auto& p : particles) {
        float px = p.pos.x * size;
        float py = p.pos.y * size;

        // U-VELOCITIES
        float ux = px;
        float uy = py - 0.5f;
        
        int ui = (int)ux;
        int uj = (int)uy;
        float uwx = ux - ui;
        float uwy = uy - uj;

        // Bilinear weights for U
        float weights_u[4] = {
            (1 - uwx) * (1 - uwy), // (i, j)
            uwx * (1 - uwy),       // (i+1, j)
            (1 - uwx) * uwy,       // (i, j+1)
            uwx * uwy              // (i+1, j+1)
        };

        // Transfer U
        int u_indices[4] = { grid.uIdx(ui, uj), grid.uIdx(ui+1, uj), grid.uIdx(ui, uj+1), grid.uIdx(ui+1, uj+1) };
        for(int k=0; k<4; ++k) {
            if (u_indices[k] < 0 || u_indices[k] >= grid.total_size) continue;
            grid.us[u_indices[k]] += weights_u[k] * p.vel.x;
            grid.uMasses[u_indices[k]] += weights_u[k];
        }

        // V-VELOCITIES
        float vx = px - 0.5f;
        float vy = py;

        int vi = (int)vx;
        int vj = (int)vy;
        float vwx = vx - vi;
        float vwy = vy - vj;

        // Bilinear weights for V
        float weights_v[4] = {
            (1 - vwx) * (1 - vwy), // (i, j)
            vwx * (1 - vwy),       // (i+1, j)
            (1 - vwx) * vwy,       // (i, j+1)
            vwx * vwy              // (i+1, j+1)
        };

        // Transfer V
        int v_indices[4] = { grid.vIdx(vi, vj), grid.vIdx(vi+1, vj), grid.vIdx(vi, vj+1), grid.vIdx(vi+1, vj+1) };
        for(int k=0; k<4; ++k) {
            if (v_indices[k] < 0 || v_indices[k] >= grid.total_size) continue;
            grid.vs[v_indices[k]] += weights_v[k] * p.vel.y;
            grid.vMasses[v_indices[k]] += weights_v[k];
        }

        classifyCells();

    }

    // Normalize
    for (int k = 0; k < grid.us.size(); k++) {
        if (grid.uMasses[k] > 1e-9f) grid.us[k] /= grid.uMasses[k];
    }
    for (int k = 0; k < grid.vs.size(); k++) {
        if (grid.vMasses[k] > 1e-9f) grid.vs[k] /= grid.vMasses[k];
    }
}

void Simulation::applyForces(float dt) {
    for (int k = 0; k < (size+1)*size; k++) {
        // if (grid.vMasses[k] > 1e-2)
        grid.new_vs[k] = grid.vs[k] + GRAVITY * dt;
        grid.new_us[k] = grid.us[k];
    }

    // for (int j = 0; j < size/2; j++) {
    //     int idx = grid.vIdx(size * 6 / 8,j);
    //     grid.new_vs[idx] += 1.0f;
    // }
}

void Simulation::computeDivergences(float dt) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (grid.cellType[grid.gridIdx(i, j)] == CellType::SOLID) {
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
            if (grid.cellType[grid.gridIdx(i, j)] == CellType::SOLID) {
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

void Simulation::solvePressure(float dt) {
    int n = grid.total_size;
    std::vector<float> r(n, 0.0f);
    std::vector<float> d(n, 0.0f);
    std::vector<float> Ad(n, 0.0f);
    std::vector<float> b(n, 0.0f);

    for (int i = 0; i < n; i++) {
        if (grid.cellType[i] == CellType::FLUID) {
            b[i] = -h * h * grid.divergence[i];
        } else {
            b[i] = 0.0f;
        }
        grid.pressure[i] = 0.0f;
    }

    // Initial residual is b since p_0 = 0
    r = b; 
    d = r;
    
    float deltaNew = dotProduct(r, r);
    float epsilon = 1e-6f;
    int maxIter = 100;

    for (int k = 0; k < maxIter; k++) {
        if (deltaNew < epsilon)  {
            if (dt >= 0.02f)
            std::cout << "CG converged in " << k << " iterations. Residual: " << deltaNew << " Also dt = " << dt << std::endl;
            break;
        }

        applyA(d, Ad);

        float dAd = dotProduct(d, Ad);
        float alpha = (dAd < 1e-6) ? 0.0f : (deltaNew / dAd);

        for (int i = 0; i < n; i++) {
            grid.pressure[i] += alpha * d[i];
            r[i] -= alpha * Ad[i];
        }

        float deltaOld = deltaNew;
        deltaNew = dotProduct(r, r);

        float beta = (float)(deltaNew / deltaOld);

        for (int i = 0; i < n; i++) {
            d[i] = r[i] + beta * d[i];
        }
    }
}

void Simulation::applyPressure(float dt) {
    float K = dt / RHO / h;

    // U VELOCITIES
    for (int i = 1; i < size; i++) {
        for (int j = 1; j < size - 1; j++) {
            int idxLeft = grid.gridIdx(i - 1, j);
            int idxRight = grid.gridIdx(i, j);
            int uIdx = grid.uIdx(i, j);

            if ((grid.cellType[idxLeft] == CellType::FLUID || grid.cellType[idxRight] == CellType::FLUID) &&
                (grid.cellType[idxLeft] != CellType::SOLID && grid.cellType[idxRight] != CellType::SOLID)) {

                grid.new_us[uIdx] -= K * (grid.pressure[idxRight] - grid.pressure[idxLeft]);
            } else if (grid.cellType[idxLeft] == CellType::SOLID || grid.cellType[idxRight] == CellType::SOLID) {
                grid.new_us[uIdx] = 0.0f;
            }
        }
    }

    // V VELOCITIES
    for (int i = 0; i < size; i++) {
        for (int j = 1; j < size; j++) {
            int idxBot = grid.gridIdx(i, j - 1);
            int idxTop = grid.gridIdx(i, j);
            int vIdx = grid.vIdx(i, j);

            if ((grid.cellType[idxBot] == CellType::FLUID || grid.cellType[idxTop] == CellType::FLUID) &&
                (grid.cellType[idxBot] != CellType::SOLID && grid.cellType[idxTop] != CellType::SOLID)) {
                
                grid.new_vs[vIdx] -= K * (grid.pressure[idxTop] - grid.pressure[idxBot]);
            }
            else if (grid.cellType[idxBot] == CellType::SOLID || grid.cellType[idxTop] == CellType::SOLID) {
                grid.new_vs[vIdx] = 0.0f;
            }
        }
    }

    computeDivergences(dt); // just to debug
}

void Simulation::g2p(float dt) {
    float alpha = 0.95f; // Flip ratio

    for (auto& p : particles) {
        // p.vel = glm::vec2(0.0f);

        float px = p.pos.x * size;
        float py = p.pos.y * size;

        // U-VELOCITIES
        float ux = px;
        float uy = py - 0.5f;
        
        int ui = (int)ux;
        int uj = (int)uy;
        float uwx = ux - ui;
        float uwy = uy - uj;

        // Bilinear weights for U
        float weights_u[4] = {
            (1 - uwx) * (1 - uwy), // (i, j)
            uwx * (1 - uwy),       // (i+1, j)
            (1 - uwx) * uwy,       // (i, j+1)
            uwx * uwy              // (i+1, j+1)
        };
        int u_indices[4] = { grid.uIdx(ui, uj), grid.uIdx(ui+1, uj), grid.uIdx(ui, uj+1), grid.uIdx(ui+1, uj+1) };
        // Transfer U
        float pic_u = 0.0f;
        float flip_u = p.vel.x;
        
        for(int k=0; k<4; ++k) {
            if (u_indices[k] < 0 || u_indices[k] >= grid.total_size) continue;
            float new_u = grid.new_us[u_indices[k]];
            float old_u = grid.us[u_indices[k]];

            pic_u += new_u * weights_u[k];
            flip_u += (new_u - old_u) * weights_u[k];
        }

        // V-VELOCITIES
        float vx = px - 0.5f;
        float vy = py;

        int vi = (int)vx;
        int vj = (int)vy;
        float vwx = vx - vi;
        float vwy = vy - vj;

        // Bilinear weights for V
        float weights_v[4] = {
            (1 - vwx) * (1 - vwy), // (i, j)
            vwx * (1 - vwy),       // (i+1, j)
            (1 - vwx) * vwy,       // (i, j+1)
            vwx * vwy              // (i+1, j+1)
        };
        int v_indices[4] = { grid.vIdx(vi, vj), grid.vIdx(vi+1, vj), grid.vIdx(vi, vj+1), grid.vIdx(vi+1, vj+1) };
        
        // Transfer V
        float pic_v = 0.0f;
        float flip_v = p.vel.y;
        
        for(int k=0; k<4; ++k) {
            if (v_indices[k] < 0 || v_indices[k] >= grid.total_size) continue;
            float new_v = grid.new_vs[v_indices[k]];
            float old_v = grid.vs[v_indices[k]];

            pic_v += new_v * weights_v[k];
            flip_v += (new_v - old_v) * weights_v[k];
        }

        p.vel.x = (1.0f - alpha) * pic_u + alpha * flip_u;
        p.vel.y = (1.0f - alpha) * pic_v + alpha * flip_v;

        p.pos += p.vel * dt;

        // Collision
        int cellX = (int)(p.pos.x * size);
        int cellY = (int)(p.pos.y * size);

        cellX = std::max(0, std::min(size - 1, cellX));
        cellY = std::max(0, std::min(size - 1, cellY));

        if (grid.cellType[grid.gridIdx(cellX, cellY)] == CellType::SOLID) {
            float cellMinX = (float)cellX / size;
            float cellMaxX = (float)(cellX + 1) / size;
            float cellMinY = (float)cellY / size;
            float cellMaxY = (float)(cellY + 1) / size;

            // The 2.0f is a safety to ensure the particle does not try to get out of the box
            float distLeft   = (p.pos.x < h        || grid.cellType[grid.gridIdx(cellX-1, cellY)] == CellType::SOLID) ? 2.0f : p.pos.x - cellMinX;
            float distRight  = (p.pos.x > 1.0f - h || grid.cellType[grid.gridIdx(cellX+1, cellY)] == CellType::SOLID) ? 2.0f : cellMaxX - p.pos.x;
            float distBottom = (p.pos.y < h        || grid.cellType[grid.gridIdx(cellX, cellY-1)] == CellType::SOLID) ? 2.0f : p.pos.y - cellMinY;
            float distTop    = (p.pos.y > 1.0f - h || grid.cellType[grid.gridIdx(cellX, cellY+1)] == CellType::SOLID) ? 2.0f : cellMaxY - p.pos.y;

            float minDist = distLeft;
            int side = 0;

            if (distRight < minDist)  { minDist = distRight;  side = 1; }
            if (distBottom < minDist) { minDist = distBottom; side = 2; }
            if (distTop < minDist)    { minDist = distTop;    side = 3; }

            float eps = 1e-4f;

            switch (side) {
                case 0: // Left edge
                    p.pos.x = cellMinX - eps;
                    p.vel.x = 0; 
                    break;
                case 1: // Right edge
                    p.pos.x = cellMaxX + eps;
                    p.vel.x = 0;
                    break;
                case 2: // Bottom edge
                    p.pos.y = cellMinY - eps;
                    p.vel.y = 0;
                    break;
                case 3: // Top edge
                    p.pos.y = cellMaxY + eps;
                    p.vel.y = 0;
                    break;
            }
            p.pos.x = std::min(1.0f - h, std::max(h, p.pos.x));
            p.pos.y = std::min(1.0f - h, std::max(h, p.pos.y));
        }
    }
}


// HELPERS

void Simulation::applyA(const std::vector<float>& x, std::vector<float>& Ax) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            int idx = grid.gridIdx(i, j);
            if (grid.cellType[idx] != CellType::FLUID) {
                Ax[idx] = 0.0f;
                continue;
            }

            float val = 0.0f;
            int neighborsCount = 0;
            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};

            for (auto& n : neighbors) {
                int ni = n[0], nj = n[1];
                // Boundary check
                if (ni >= 0 && ni < size && nj >= 0 && nj < size) {
                    int nIdx = grid.gridIdx(ni, nj);

                    if (grid.cellType[nIdx] != CellType::SOLID) {
                        neighborsCount++;
                        if (grid.cellType[nIdx] == CellType::FLUID) val -= x[nIdx];
                    }
                }
            }
            Ax[idx] = (neighborsCount * x[idx]) + val;
        }
    }
}

float Simulation::dotProduct(const std::vector<float>& a, const std::vector<float>& b) {
    float result = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        result += a[i] * b[i];
    }
    return result;
}

void Simulation::classifyCells() {
    // reset
    for(int i=0; i < grid.total_size; ++i) {
        if(grid.cellType[i] != CellType::SOLID) grid.cellType[i] = CellType::AIR;
    }

    // mark cells
    for (auto& p : particles) {
        int i = (int)(p.pos.x * size);
        int j = (int)(p.pos.y * size);
        int idx = grid.gridIdx(i, j);
        if (grid.cellType[idx] == CellType::AIR) grid.cellType[idx] = CellType::FLUID;
    }
}

void Simulation::addParticle(glm::vec2 pos) {
    int i = (int)(pos.x*size);
    int j = (int)(pos.y*size);
    if (grid.cellType[grid.gridIdx(i,j)] == CellType::SOLID) return;
    glm::vec2 vel((float)rand()/RAND_MAX-0.5f, 0.0f);
    particles.emplace_back(pos, vel);
}