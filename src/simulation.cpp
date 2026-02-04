#include "simulation.h"
#include "shader.h"


// === MAIN STEPS ===

void Simulation::p2g(float dt) {
    clearBuffer(uSSBO);
    clearBuffer(vSSBO);
    clearBuffer(uMassSSBO);
    clearBuffer(vMassSSBO);

    // Pass the particle velocies to the grid
    dispatchCompute(p2gProg, ((int)particles.size() + 255) / 256);

    // Normalize and apply gravity
    glUseProgram(normalizeProg);
    glUniform1f(glGetUniformLocation(normalizeProg, "dt"), dt);
    dispatchCompute(normalizeProg, NUM_GROUP_1D);

    dispatchCompute(resetCellTypesProg, NUM_GROUP_2D, NUM_GROUP_3D, NUM_GROUP_3D);

    dispatchCompute(classifyCellsProg, ((int)particles.size() + 255) / 256);
    getDataFromGPU(cellTypeSSBO, grid.cellType);
}

void Simulation::computeDivergences(float dt) {
    glUseProgram(computeDivProg);
    glUniform1f(glGetUniformLocation(computeDivProg, "dt"), dt);
    dispatchCompute(computeDivProg, NUM_GROUP_2D, NUM_GROUP_3D, NUM_GROUP_3D);
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
    int maxIter = 100;

    for (int k = 0; k < maxIter; k++) {
        // Compute Ad        
        dispatchCompute(applyAProg, NUM_GROUP_2D, NUM_GROUP_3D, NUM_GROUP_3D);

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

    dispatchCompute(applyPressureProg, NUM_GROUP_2D, NUM_GROUP_3D, NUM_GROUP_3D); 
    // Theoretically i should also got to size+1 but since these are zero velocities, i don't care
}

void Simulation::g2p(float dt) {
    glUseProgram(g2pProg);
    glUniform1f(glGetUniformLocation(g2pProg, "dt"), dt);

    dispatchCompute(g2pProg, ((int)particles.size() + 255) / 256);
    getDataFromGPU(particleSSBO, particles);
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
    if (i<1 || i>=size-1 || j<1 || j>=size-1) return;
    if (grid.cellType[grid.gridIdx(i,j)] == CellType::SOLID) return;

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

    initBuffer(13, residualSSBO, residual);
    initBuffer(14, directionSSBO, direction);
    initBuffer(15, adSSBO, Ad); 

    glGenBuffers(1, &paramsSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, paramsSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 4*sizeof(float), &params, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 16, paramsSSBO);

    // Compile shaders
    p2gProg = createShaderProgram({{"shaders/compute/p2g.glsl", ShaderType::COMPUTE}});
    normalizeProg = createShaderProgram({{"shaders/compute/normalizeAndApplyForces.glsl", ShaderType::COMPUTE}});
    resetCellTypesProg = createShaderProgram({{"shaders/compute/resetCellTypes.glsl", ShaderType::COMPUTE}});
    classifyCellsProg = createShaderProgram({{"shaders/compute/classifyCells.glsl", ShaderType::COMPUTE}});

    computeDivProg = createShaderProgram({{"shaders/compute/computeDiv.glsl", ShaderType::COMPUTE}});

    dotProductProg = createShaderProgram({{"shaders/compute/dotProduct.glsl", ShaderType::COMPUTE}});
    moveAlphaProg = createShaderProgram({{"shaders/compute/moveGradientAlpha.glsl", ShaderType::COMPUTE}});
    moveBetaProg = createShaderProgram({{"shaders/compute/moveGradientBeta.glsl", ShaderType::COMPUTE}});
    transitionProg = createShaderProgram({{"shaders/compute/transition.glsl", ShaderType::COMPUTE}});

    applyAProg = createShaderProgram({{"shaders/compute/applyA.glsl", ShaderType::COMPUTE}});

    applyPressureProg = createShaderProgram({{"shaders/compute/applyPressure.glsl", ShaderType::COMPUTE}});
    g2pProg = createShaderProgram({{"shaders/compute/g2p.glsl", ShaderType::COMPUTE}});


    // Init unifrom values
    glUseProgram(p2gProg);
    glUniform1i(glGetUniformLocation(p2gProg, "numParticles"), (int)particles.size());
    glUniform1i(glGetUniformLocation(p2gProg, "size"), size);

    glUseProgram(normalizeProg);
    glUniform1i(glGetUniformLocation(normalizeProg, "size"), size);
    glUniform1f(glGetUniformLocation(normalizeProg, "gravity"), GRAVITY);

    glUseProgram(classifyCellsProg);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "size"), size);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "numParticles"), (int)particles.size());

    glUseProgram(resetCellTypesProg);
    glUniform1i(glGetUniformLocation(resetCellTypesProg, "size"), size);

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

    sendDataToGPU(cellTypeSSBO, grid.cellType);
    sendDataToGPU(particleSSBO, particles);
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
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleSSBO); // Re-link to binding 0
    
    glUseProgram(p2gProg);
    glUniform1i(glGetUniformLocation(p2gProg, "numParticles"), (int)particles.size());
    glUseProgram(g2pProg);
    glUniform1i(glGetUniformLocation(g2pProg, "numParticles"), (int)particles.size());
    glUseProgram(classifyCellsProg);
    glUniform1i(glGetUniformLocation(classifyCellsProg, "numParticles"), (int)particles.size());

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