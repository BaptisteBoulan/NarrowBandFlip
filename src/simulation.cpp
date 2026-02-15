#include "simulation.h"
#include "shader.h"


// === MAIN STEPS ===

void Simulation::p2g(float dt) {
    glUseProgram(advectGridProg);
    glUniform1f(glGetUniformLocation(advectGridProg, "dt"), dt);
    dispatchCompute(advectGridProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);

    clearBuffer(uSSBO);
    clearBuffer(vSSBO);
    clearBuffer(wSSBO);

    clearBuffer(uMassSSBO);
    clearBuffer(vMassSSBO);
    clearBuffer(wMassSSBO);

    // Pass the particle velocies to the grid
    dispatchCompute(p2gProg, NUM_PARTICLES_GROUP);

    // Normalize and apply gravity
    glUseProgram(normalizeProg);
    glUniform1f(glGetUniformLocation(normalizeProg, "dt"), dt);
    dispatchCompute(normalizeProg, NUM_VELOCITIES);

    dispatchCompute(resetCellTypesProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);

    dispatchCompute(classifyCellsProg, NUM_PARTICLES_GROUP);
    getDataFromGPU(cellTypeSSBO, grid.cellType);

    clearBufferInt(particlesLevelSetSSBO, 0);
    dispatchCompute(particleLevelSetProg, NUM_PARTICLES_GROUP);
    dispatchCompute(updateLevelSetProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);

    // Propagate depth: iterate size/3 times
    const int numRedistancePasses = size / 3;
    for (int i = 0; i < numRedistancePasses; ++i) {
        // Propagate distances: read finalLevelSet(17), write newLevelSet(14)
        dispatchCompute(redistanceProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);

        // Copy result back for next iteration: newLevelSet(14) -> finalLevelSet(17)
        glCopyNamedBufferSubData(newLevelSetSSBO, finalLevelSetSSBO, 0, 0, grid.finalLevelSet.size() * sizeof(float));
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }

    // Copy to main levelSet buffer
    glCopyNamedBufferSubData(finalLevelSetSSBO, levelSetSSBO, 0, 0, grid.levelSet.size() * sizeof(float));
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    cullAndResample();
}

void Simulation::computeDivergences(float dt) {
    glUseProgram(computeDivProg);
    glUniform1f(glGetUniformLocation(computeDivProg, "dt"), dt);
    dispatchCompute(computeDivProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);
    getDataFromGPU(divSSBO, grid.divergence);
}

void Simulation::solvePressure(float dt) {
    std::vector<float> residual(grid.total_cells, 0.0f);
    std::vector<float> b(grid.total_cells, 0.0f);

    direction.assign(grid.total_cells, 0.0f);   
    Ad.assign(grid.total_cells, 0.0f); 
    params.reset();

    for (int k = 0; k < grid.total_cells; k++) {
        if (grid.cellType[k] == CellType::FLUID) {
            b[k] = -h * h * grid.divergence[k];
        } else {
            b[k] = 0.0f;
        }
        grid.pressure[k] = 0.0f;
    }

    // r_0 is b since p_0 = 0
    residual = b; 
    direction = residual;

    sendDataToGPU(residualSSBO, residual);
    sendDataToGPU(pressureSSBO, grid.pressure);
    sendDataToGPU(directionSSBO, direction);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, paramsSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 4*sizeof(float), &params);

    glUseProgram(dotProductProg);
    glUniform1i(glGetUniformLocation(dotProductProg, "computeDelta"), 1);
    dispatchCompute(dotProductProg, NUM_GROUP_1D);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, paramsSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 4*sizeof(float), &params);

    float epsilon = 1e-6f;
    int maxIter = 200;

    for (int k = 0; k < maxIter; k++) {
        // Compute Ad        
        dispatchCompute(applyAProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D);

        // Compute dAd
        glUseProgram(dotProductProg);
        glUniform1i(glGetUniformLocation(dotProductProg, "computeDelta"), 0);
        dispatchCompute(dotProductProg, NUM_GROUP_1D);

        // Compute alpha, passes deltaNew to deltaOld and reset delta new and dad
        dispatchCompute(transitionProg);

        // Update pressure and residual
        dispatchCompute(moveAlphaProg, NUM_GROUP_1D);

        // Compute deltaNew
        glUseProgram(dotProductProg);
        glUniform1i(glGetUniformLocation(dotProductProg, "computeDelta"), 1);
        dispatchCompute(dotProductProg, NUM_GROUP_1D);

        // Update direction
        dispatchCompute(moveBetaProg, NUM_GROUP_1D);
    }
}

void Simulation::applyPressure(float dt) {
    float K = dt / RHO / h;

    glUseProgram(applyPressureProg);
    glUniform1f(glGetUniformLocation(applyPressureProg, "K"),K);

    dispatchCompute(applyPressureProg, NUM_GROUP_3D, NUM_GROUP_3D, NUM_GROUP_3D); 
    // Theoretically i should also got to size+1 but since these are zero velocities, i don't care
}

void Simulation::g2p(float dt) {
    glUseProgram(g2pProg);
    glUniform1f(glGetUniformLocation(g2pProg, "dt"), dt);

    dispatchCompute(g2pProg, NUM_PARTICLES_GROUP);
    getDataFromGPU(particleSSBO, particles);

    clearBufferInt(densitySSBO, 0);
    dispatchCompute(computeDensityProg, NUM_PARTICLES_GROUP);
    getDataFromGPU(densitySSBO, density);
}


// HELPERS

float Simulation::dotProduct(const std::vector<float>& a, const std::vector<float>& b) {
    float result = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        result += a[i] * b[i];
    }
    return result;
}

void Simulation::addParticle(glm::vec3 pos) {
    int i = (int)(pos.x*size);
    int j = (int)(pos.y*size);
    int k = (int)(pos.z*size);
    if (i<1 || i>=size-1 || j<1 || j>=size-1 || k<1 || k>=size-1) return;
    if (grid.cellType[grid.gridIdx(i,j,k)] == CellType::SOLID) return;
    

    float theta = (float)(2 * M_PI * rand())/RAND_MAX;
    float rho = 0.03f * (float)rand()/RAND_MAX;
    glm::vec3 offset(cos(theta), sin(theta), 0.0f);
    particles.emplace_back(pos + rho * offset);
}


// === GPU ===

template<typename T>
void Simulation::initBuffer(int index, GLuint& ssbo, const std::vector<T>& data) {
    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, data.size() * sizeof(T), data.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
}

void Simulation::initGPU() {

    initBuffer(0, particleSSBO, particles);

    initBuffer(1, uSSBO, grid.us);
    initBuffer(2, vSSBO, grid.vs);
    initBuffer(3, wSSBO, grid.ws);

    initBuffer(4, uMassSSBO, grid.uMasses);
    initBuffer(5, vMassSSBO, grid.vMasses);
    initBuffer(6, wMassSSBO, grid.wMasses);

    initBuffer(7, newuSSBO, grid.new_us);
    initBuffer(8, newvSSBO, grid.new_vs);
    initBuffer(9, newwSSBO, grid.new_ws);

    initBuffer(10, cellTypeSSBO, grid.cellType);
    initBuffer(11, divSSBO, grid.divergence);
    initBuffer(12, pressureSSBO, grid.pressure);

    initBuffer(13, levelSetSSBO, grid.levelSet);
    initBuffer(14, newLevelSetSSBO, grid.newLevelSet);
    initBuffer(16, particlesLevelSetSSBO, grid.particlesLevelSet);
    initBuffer(17, finalLevelSetSSBO, grid.finalLevelSet);

    initBuffer(18, residualSSBO, residual);
    initBuffer(19, directionSSBO, direction);
    initBuffer(20, adSSBO, Ad); 

    initBuffer(22, uAdvSSBO, grid.u_adv);
    initBuffer(23, vAdvSSBO, grid.v_adv);
    initBuffer(24, wAdvSSBO, grid.w_adv);

    initBuffer(25, densitySSBO, density);

    glGenBuffers(1, &paramsSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, paramsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 4*sizeof(float), &params, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 21, paramsSSBO);
    

    // Compile shaders
    advectGridProg          = createShaderProgram({{"shaders/compute/advectGrid.glsl",                  ShaderType::COMPUTE}});
    p2gProg                 = createShaderProgram({{"shaders/compute/p2g.glsl",                         ShaderType::COMPUTE}});
    normalizeProg           = createShaderProgram({{"shaders/compute/normalizeAndApplyForces.glsl",     ShaderType::COMPUTE}});
    resetCellTypesProg      = createShaderProgram({{"shaders/compute/resetCellTypes.glsl",              ShaderType::COMPUTE}});
    classifyCellsProg       = createShaderProgram({{"shaders/compute/classifyCells.glsl",               ShaderType::COMPUTE}});
    updateLevelSetProg      = createShaderProgram({{"shaders/compute/updateLevelSet.glsl",              ShaderType::COMPUTE}});
    redistanceProg          = createShaderProgram({{"shaders/compute/redistance.glsl",                  ShaderType::COMPUTE}});
    particleLevelSetProg    = createShaderProgram({{"shaders/compute/particleLevelSet.glsl",            ShaderType::COMPUTE}});

    computeDivProg          = createShaderProgram({{"shaders/compute/computeDiv.glsl",                  ShaderType::COMPUTE}});

    dotProductProg          = createShaderProgram({{"shaders/compute/dotProduct.glsl",                  ShaderType::COMPUTE}});
    moveAlphaProg           = createShaderProgram({{"shaders/compute/moveGradientAlpha.glsl",           ShaderType::COMPUTE}});
    moveBetaProg            = createShaderProgram({{"shaders/compute/moveGradientBeta.glsl",            ShaderType::COMPUTE}});
    transitionProg          = createShaderProgram({{"shaders/compute/transition.glsl",                  ShaderType::COMPUTE}});

    applyAProg              = createShaderProgram({{"shaders/compute/applyA.glsl",                      ShaderType::COMPUTE}});

    applyPressureProg       = createShaderProgram({{"shaders/compute/applyPressure.glsl",               ShaderType::COMPUTE}});
    g2pProg                 = createShaderProgram({{"shaders/compute/g2p.glsl",                         ShaderType::COMPUTE}});

    computeDensityProg      = createShaderProgram({{"shaders/compute/computeDensity.glsl",              ShaderType::COMPUTE}});


    // Init unifrom values
    glUseProgram(p2gProg);
    glUniform1i(glGetUniformLocation(p2gProg, "numParticles"), (int)particles.size());
    glUniform1i(glGetUniformLocation(p2gProg, "size"), size);

    glUseProgram(normalizeProg);
    glUniform1i(glGetUniformLocation(normalizeProg, "size"), size);
    glUniform1f(glGetUniformLocation(normalizeProg, "gravity"), GRAVITY);
    glUniform1f(glGetUniformLocation(normalizeProg, "h"), h);

    glUseProgram(classifyCellsProg);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "size"), size);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "numParticles"), (int)particles.size());

    glUseProgram(resetCellTypesProg);
    glUniform1i(glGetUniformLocation(resetCellTypesProg, "size"), size);

    glUseProgram(updateLevelSetProg);
    glUniform1i(glGetUniformLocation(updateLevelSetProg, "size"), size);

    glUseProgram(redistanceProg);
    glUniform1i(glGetUniformLocation(redistanceProg, "size"), size);

    glUseProgram(particleLevelSetProg);
    glUniform1i(glGetUniformLocation(particleLevelSetProg, "size"), size);
    glUniform1i(glGetUniformLocation(particleLevelSetProg, "numParticles"), (int)particles.size());

    glUseProgram(computeDivProg);
    glUniform1i(glGetUniformLocation(computeDivProg, "size"), size);
    glUniform1f(glGetUniformLocation(computeDivProg, "h"), h);
    glUniform1f(glGetUniformLocation(computeDivProg, "rho"), RHO);

    glUseProgram(applyPressureProg);
    glUniform1i(glGetUniformLocation(applyPressureProg, "size"), size);

    glUseProgram(applyAProg);
    glUniform1i(glGetUniformLocation(applyAProg, "size"), size);
    
    glUseProgram(dotProductProg);
    glUniform1i(glGetUniformLocation(dotProductProg, "size"), size);
    
    glUseProgram(moveAlphaProg);
    glUniform1i(glGetUniformLocation(moveAlphaProg, "size"), size);
    
    glUseProgram(moveBetaProg);
    glUniform1i(glGetUniformLocation(moveBetaProg, "size"), size);

    glUseProgram(g2pProg);
    glUniform1i(glGetUniformLocation(g2pProg, "size"), size);
    glUniform1i(glGetUniformLocation(g2pProg, "numParticles"), (int)particles.size());
    glUniform1f(glGetUniformLocation(g2pProg, "h"), h);
    glUniform1f(glGetUniformLocation(g2pProg, "alpha"), ALPHA);

    glUseProgram(advectGridProg);
    glUniform1i(glGetUniformLocation(advectGridProg, "size"), size);
    glUniform1f(glGetUniformLocation(advectGridProg, "h"), h);

    glUseProgram(computeDensityProg);
    glUniform1i(glGetUniformLocation(computeDensityProg, "size"), size);


    sendDataToGPU(cellTypeSSBO, grid.cellType);
    sendDataToGPU(particleSSBO, particles);
    sendDataToGPU(levelSetSSBO, grid.levelSet);
    sendDataToGPU(levelSetSSBO, grid.particlesLevelSet);
    sendDataToGPU(densitySSBO, density);
}



template<typename T>
void Simulation::sendDataToGPU(GLuint& ssbo, const std::vector<T>& data) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, data.size() * sizeof(T), data.data());

}
template<typename T>
void Simulation::getDataFromGPU(GLuint& ssbo, std::vector<T>& data) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, data.size() * sizeof(T), data.data());
}

void Simulation::updateParticleBuffer() {
    NUM_PARTICLES_GROUP = ((int)particles.size() + 255) / 256;

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleSSBO); // Re-link to binding 0
    
    // Update all shaders that depend on numParticles
    glUseProgram(p2gProg);
    glUniform1i(glGetUniformLocation(p2gProg, "numParticles"), (int)particles.size());
    
    glUseProgram(g2pProg);
    glUniform1i(glGetUniformLocation(g2pProg, "numParticles"), (int)particles.size());
    
    glUseProgram(classifyCellsProg);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "numParticles"), (int)particles.size());
    
    glUseProgram(particleLevelSetProg);
    glUniform1i(glGetUniformLocation(particleLevelSetProg, "numParticles"), (int)particles.size());
    
    glUseProgram(computeDensityProg);
    glUniform1i(glGetUniformLocation(computeDensityProg, "numParticles"), (int)particles.size());
}

void Simulation::dispatchCompute(GLuint prog, int numX, int numY, int numZ) {
    glUseProgram(prog);
    glDispatchCompute(numX, numY, numZ);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void Simulation::clearBuffer(GLuint buffer) {
    float zero = 0.0f;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);
    glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32F, GL_RED, GL_FLOAT, &zero);
}

void Simulation::clearBufferInt(GLuint buffer, int value) {
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, buffer);
    glClearBufferData(GL_SHADER_STORAGE_BUFFER, GL_R32I, GL_RED_INTEGER, GL_INT, &value);
}

// === PARTICLE MANAGEMENT ===

void Simulation::cullAndResample() {
    getDataFromGPU(levelSetSSBO, grid.levelSet);
    getDataFromGPU(finalLevelSetSSBO, grid.finalLevelSet);
    
    getDataFromGPU(newuSSBO, grid.new_us);
    getDataFromGPU(newvSSBO, grid.new_vs);
    getDataFromGPU(newwSSBO, grid.new_ws);

    std::vector<Particle> keptParticles;
    keptParticles.reserve(particles.size());

    float threshold = 8.0f * h;
    
    std::vector<int> cellCounts(grid.total_cells, 0);

    for (const auto& p : particles) {
        int i = (int)(p.pos.x * size);
        int j = (int)(p.pos.y * size);
        int k = (int)(p.pos.z * size);

        int idx = grid.gridIdx(i,j,k);
        float phi = grid.finalLevelSet[idx];
        
        if (phi <= threshold) {
            keptParticles.push_back(p);
            
            int i = (int)(p.pos.x * size);
            int j = (int)(p.pos.y * size);
            int k = (int)(p.pos.z * size);
            
            if (i >= 0 && i < size && j >= 0 && j < size && k >= 0 && k < size) {
                int idx = grid.gridIdx(i, j, k);
                cellCounts[idx]++;
            }
        }
    }

    int particlesAdded = 0;
    
    // Iterate over all cells
    for (int k = 0; k < size; ++k) {
        for (int j = 0; j < size; ++j) {
            for (int i = 0; i < size; ++i) {
                int idx = grid.gridIdx(i, j, k);
                float phi = grid.levelSet[idx]; // Cell center value

                if (phi > threshold - 3*h && phi < threshold) {
                    int count = cellCounts[idx];
                    if (count < 20) {
                        int toAdd = 20 - count;
                        for (int n = 0; n < toAdd; ++n) {
                            // Generate random position within the cell
                            // cell range: [i/size, (i+1)/size]
                            float rx = (float)rand() / RAND_MAX;
                            float ry = (float)rand() / RAND_MAX;
                            float rz = (float)rand() / RAND_MAX;
                            
                            glm::vec3 pos((i + rx) / size, (j + ry) / size, (k + rz) / size);
                            
                            // Initialize new particle
                            Particle newP(pos);
                            glm::vec3 vel = sampleVelocity(pos);
                            newP.vel = glm::vec4(vel, 0.0f);
                            
                            keptParticles.push_back(newP);
                            particlesAdded++;
                        }
                    }
                }
            }
        }
    }

    particles = std::move(keptParticles);
    
    updateParticleBuffer();
}

glm::vec3 Simulation::sampleVelocity(glm::vec3 pos) {
    float px = pos.x * size;
    float py = pos.y * size;
    float pz = pos.z * size;
    
    // --- U Sample ---    
    float u_val = 0.0f;
    {
        float ux = px;
        float uy = py - 0.5f;
        float uz = pz - 0.5f;
        
        int i = (int)floor(ux);
        int j = (int)floor(uy);
        int k = (int)floor(uz);
        
        float tx = ux - i;
        float ty = uy - j;
        float tz = uz - k;
        
        for (int kk = 0; kk <= 1; ++kk) {
            for (int jj = 0; jj <= 1; ++jj) {
                for (int ii = 0; ii <= 1; ++ii) {
                    int nx = std::max(0, std::min(size, i + ii));
                    int ny = std::max(0, std::min(size - 1, j + jj));
                    int nz = std::max(0, std::min(size - 1, k + kk));
                    
                    int idx = grid.uIdx(nx, ny, nz);
                    float weight = (ii == 0 ? (1.0f - tx) : tx) *
                                   (jj == 0 ? (1.0f - ty) : ty) *
                                   (kk == 0 ? (1.0f - tz) : tz);
                                   
                    u_val += grid.new_us[idx] * weight;
                }
            }
        }
    }

    // --- V Sample ---
    float v_val = 0.0f;
    {
        float vx = px - 0.5f;
        float vy = py;
        float vz = pz - 0.5f;
        
        int i = (int)floor(vx);
        int j = (int)floor(vy);
        int k = (int)floor(vz);
        
        float tx = vx - i;
        float ty = vy - j;
        float tz = vz - k;
        
        for (int kk = 0; kk <= 1; ++kk) {
            for (int jj = 0; jj <= 1; ++jj) {
                for (int ii = 0; ii <= 1; ++ii) {
                    int nx = std::max(0, std::min(size - 1, i + ii));
                    int ny = std::max(0, std::min(size, j + jj));
                    int nz = std::max(0, std::min(size - 1, k + kk));
                    
                    int idx = grid.vIdx(nx, ny, nz);
                    float weight = (ii == 0 ? (1.0f - tx) : tx) *
                                   (jj == 0 ? (1.0f - ty) : ty) *
                                   (kk == 0 ? (1.0f - tz) : tz);
                                   
                    v_val += grid.new_vs[idx] * weight;
                }
            }
        }
    }

    // --- W Sample ---
    float w_val = 0.0f;
    {
        float wx = px - 0.5f;
        float wy = py - 0.5f;
        float wz = pz;
        
        int i = (int)floor(wx);
        int j = (int)floor(wy);
        int k = (int)floor(wz);
        
        float tx = wx - i;
        float ty = wy - j;
        float tz = wz - k;
        
        for (int kk = 0; kk <= 1; ++kk) {
            for (int jj = 0; jj <= 1; ++jj) {
                for (int ii = 0; ii <= 1; ++ii) {
                    int nx = std::max(0, std::min(size - 1, i + ii));
                    int ny = std::max(0, std::min(size - 1, j + jj));
                    int nz = std::max(0, std::min(size, k + kk));
                    
                    int idx = grid.wIdx(nx, ny, nz);
                    float weight = (ii == 0 ? (1.0f - tx) : tx) *
                                   (jj == 0 ? (1.0f - ty) : ty) *
                                   (kk == 0 ? (1.0f - tz) : tz);
                                   
                    w_val += grid.new_ws[idx] * weight;
                }
            }
        }
    }

    return glm::vec3(u_val, v_val, w_val);
}