#include "simulation.h"
#include "shader.h"


// === MAIN STEPS ===

void Simulation::p2g() {



    p2gGPU();

    // Normalize
    for (int k = 0; k < grid.us.size(); k++) {
        if (grid.uMasses[k] > 1e-9f) grid.us[k] /= grid.uMasses[k];
    }
    for (int k = 0; k < grid.vs.size(); k++) {
        if (grid.vMasses[k] > 1e-9f) grid.vs[k] /= grid.vMasses[k];
    }

    classifyCells();
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
    std::vector<float> residual(grid.total_size, 0.0f);
    std::vector<float> b(grid.total_size, 0.0f);

    direction.assign(grid.total_size, 0.0f);   
    Ad.assign(grid.total_size, 0.0f);   



    for (int k = 0; k < grid.total_size; k++) {
        if (grid.cellType[k] == CellType::FLUID) {
            b[k] = -h * h * grid.divergence[k];
        } else {
            b[k] = 0.0f;
        }
        grid.pressure[k] = 0.0f;
    }

    // Initial residual is b since p_0 = 0
    residual = b; 
    direction = residual;
    
    float deltaNew = dotProduct(residual, residual);
    float epsilon = 1e-6f;
    int maxIter = 100;


    // Set Shader Data
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cellTypeSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.cellType.size() * sizeof(int), grid.cellType.data());

    for (int k = 0; k < maxIter; k++) {
        if (deltaNew < epsilon)  {
            if (dt >= 0.02f)
            std::cout << "CG converged in " << k << " iterations. Residual: " << deltaNew << " Also dt = " << dt << std::endl;
            break;
        }

        applyA();

        float dAd = dotProduct(direction, Ad);
        float alpha = (dAd < 1e-6) ? 0.0f : (deltaNew / dAd);

        for (int i = 0; i < grid.total_size; i++) {
            grid.pressure[i] += alpha * direction[i];
            residual[i] -= alpha * Ad[i];
        }

        float deltaOld = deltaNew;
        deltaNew = dotProduct(residual, residual);

        float beta = (float)(deltaNew / deltaOld);

        for (int i = 0; i < grid.total_size; i++) {
            direction[i] = residual[i] + beta * direction[i];
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

    g2pGPU(dt);
}


// HELPERS

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
    if (i<1 || i>=size-1 || j<1 || j>=size-1) return;
    if (grid.cellType[grid.gridIdx(i,j)] == CellType::SOLID) return;

    float theta = (float)(2 * M_PI * rand())/RAND_MAX;
    float rho = 0.03f * (float)rand()/RAND_MAX;
    glm::vec2 offset(cos(theta), sin(theta));
    particles.emplace_back(pos + rho * offset);

    updateParticleBuffer();
}


// === GPU ===

void Simulation::initGPU() {
    // Particle SSBO
    glGenBuffers(1, &particleSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleSSBO);

    // us SSBO
    glGenBuffers(1, &uSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.us.size() * sizeof(float), grid.us.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, uSSBO);

    // vs SSBO
    glGenBuffers(1, &vSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.vs.size() * sizeof(float), grid.vs.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, vSSBO);
    
    // u masses SSBO
    glGenBuffers(1, &uMassSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uMassSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.uMasses.size() * sizeof(float), grid.uMasses.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, uMassSSBO);

    // v masses SSBO
    glGenBuffers(1, &vMassSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vMassSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.vMasses.size() * sizeof(float), grid.vMasses.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, vMassSSBO);

    // new us SSBO
    glGenBuffers(1, &newuSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, newuSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.new_us.size() * sizeof(float), grid.new_us.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, newuSSBO);

    // new vs SSBO
    glGenBuffers(1, &newvSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, newvSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.new_vs.size() * sizeof(float), grid.new_vs.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, newvSSBO);
    
    // pressure SSBO
    glGenBuffers(1, &pressureSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pressureSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.pressure.size() * sizeof(float), grid.pressure.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, pressureSSBO);

    // cellType SSBO
    glGenBuffers(1, &cellTypeSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cellTypeSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.cellType.size() * sizeof(float), grid.cellType.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, cellTypeSSBO);

    // Ax SSBO
    glGenBuffers(1, &adSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, adSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, grid.cellType.size() * sizeof(float), grid.cellType.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, adSSBO);

    // direction SSBO
    glGenBuffers(1, &directionSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, directionSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, direction.size() * sizeof(float), direction.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, directionSSBO);

    // dAd SSBO
    glGenBuffers(1, &dAdSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, dAdSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, dAd.size() * sizeof(float), dAd.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 11, dAdSSBO);
    
    // Compile shaders
    p2gProg = createShaderProgram({{"shaders/computeP2G.glsl", ShaderType::COMPUTE}});
    applyAProg = createShaderProgram({{"shaders/computeApplyA.glsl", ShaderType::COMPUTE}});
    g2pProg = createShaderProgram({{"shaders/computeG2P.glsl", ShaderType::COMPUTE}});
}

void Simulation::p2gGPU() {
    // Clear data
    float zero = 0.0f;
    GLuint buffersToClear[] = { uSSBO, vSSBO, uMassSSBO, vMassSSBO };
    for (GLuint buf : buffersToClear) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, buf);
        glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32F, GL_RED, GL_FLOAT, &zero);
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, particles.size() * sizeof(Particle), particles.data());

    glUseProgram(p2gProg);
    
    // Set uniforms
    glUniform1i(glGetUniformLocation(p2gProg, "size"), size);
    glUniform1i(glGetUniformLocation(p2gProg, "numParticles"), (int)particles.size());

    int numGroups = ((int)particles.size() + 255) / 256;
    glDispatchCompute(numGroups, 1, 1);

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.us.size() * sizeof(float), grid.us.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.vs.size() * sizeof(float), grid.vs.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uMassSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.uMasses.size() * sizeof(float), grid.uMasses.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vMassSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.vMasses.size() * sizeof(float), grid.vMasses.data());
}

void Simulation::g2pGPU(float dt) {

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.us.size() * sizeof(float), grid.us.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.vs.size() * sizeof(float), grid.vs.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, uMassSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.uMasses.size() * sizeof(float), grid.uMasses.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, vMassSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.vMasses.size() * sizeof(float), grid.vMasses.data());
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, newuSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.new_us.size() * sizeof(float), grid.new_us.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, newvSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, grid.new_vs.size() * sizeof(float), grid.new_vs.data());

    glUseProgram(g2pProg);
    
    // Set uniforms
    glUniform1i(glGetUniformLocation(g2pProg, "size"), size);
    glUniform1i(glGetUniformLocation(g2pProg, "numParticles"), (int)particles.size());
    glUniform1f(glGetUniformLocation(g2pProg, "dt"), dt);
    glUniform1f(glGetUniformLocation(g2pProg, "h"), h);

    int numGroups = ((int)particles.size() + 255) / 256;
    glDispatchCompute(numGroups, 1, 1);

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, particles.size() * sizeof(Particle), particles.data());
}

void Simulation::updateParticleBuffer() {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleSSBO); // Re-link to binding 0
}


void Simulation::applyA() {
    bool useGPU = true;

    if(useGPU) {

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, directionSSBO);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, direction.size() * sizeof(float), direction.data());

        // Run the GPU code
        glUseProgram(applyAProg);
        glUniform1i(glGetUniformLocation(applyAProg, "size"), size);

        int numGroupsX = (size + 15) / 16;
        int numGroupsY = (size + 15) / 16;
        glDispatchCompute(numGroupsX, numGroupsY, 1);

        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        // Get the data
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, adSSBO);
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, Ad.size() * sizeof(float), Ad.data());
    } else {
        Ad.assign(grid.total_size, 0.0f);
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                int idx = grid.gridIdx(i, j);
                if (grid.cellType[idx] != CellType::FLUID) {
                    Ad[idx] = 0.0f;
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
                            if (grid.cellType[nIdx] == CellType::FLUID) val -= direction[nIdx];
                        }
                    }
                }
                Ad[idx] = (neighborsCount * direction[idx]) + val;
            }
        }

    }
}