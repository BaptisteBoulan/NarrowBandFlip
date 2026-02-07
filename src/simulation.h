#pragma once
#include "Grid.h"
#include "Particle.h"

struct SolverParams {
    float dAd;
    float deltaNew;
    float deltaOld;
    float alpha;

    void reset() {
        dAd = 0.0f;
        deltaNew = 0.0f;
        deltaOld = 0.0f;
        alpha = 0.0f;
    }
};

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
                for (int k = 0; k < size; k++) {
                    if (i == 0 || i == size - 1 || j == 0 || j == size - 1 || k == 0 || k == size - 1) {
                        grid.cellType[grid.gridIdx(i, j, k)] = CellType::SOLID;
                    } else {
                        grid.cellType[grid.gridIdx(i, j, k)] = CellType::AIR;
                    }
                }
            }
        }
        
        // Init particles
        glm::vec3 p1(0.6f, 0.2f, 0.1f);
        glm::vec3 p2(0.9f, 0.8f, 0.9f);
        glm::vec3 boxSize = p2 - p1;
        glm::vec3 boxCenter = 0.5f * (p1 + p2);
        float spacing = 0.2f / size;

        for(float x = p1.x; x < p2.x; x += spacing) {
            for (float y = p1.y; y < p2.y; y += spacing) {
                for (float z = p1.z; z < p2.z; z += spacing)
                    particles.emplace_back(glm::vec3(x,y,z));
            }
        }

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                for (int k = 0; k < size; k++) {
                    int idx = grid.gridIdx(i,j,k);

                    float x = (float)i/size;
                    float y = (float)j/size;
                    float z = (float)k/size;

                    glm::vec3 pos(x,y,z);
                    glm::vec3 sd = glm::abs(pos - boxCenter) - 0.5f * boxSize;

                    // Signed distance to the surface of the box
                    float maxSD = glm::max(glm::max(sd.x, sd.y), sd.z);
                    float minSD = glm::min(glm::max(sd.x, sd.y), sd.z);
                    grid.levelSet[idx] = minSD > 0.0f ? glm::length(glm::max(sd, 0.0f)) : maxSD;
                }
            }
        }

        Ad.assign(grid.total_cells, 0.0f);
        direction.assign(grid.total_cells, 0.0f);
        residual.assign(grid.total_cells, 0.0f);

        params = SolverParams({0,0,0,0});
        NUM_VELOCITIES = (size * size * (size+1) + 255) / 256;
        NUM_GROUP_1D = (grid.total_cells + 255) / 256;
        NUM_GROUP_2D = (size + 15) / 16;
        NUM_GROUP_3D = (size + 7) / 8;

        std::cout<<particles.size()<<std::endl;
    }

    // STEPS
    void p2g(float dt);
    void computeDivergences(float dt);
    void solvePressure(float dt);
    void applyPressure(float dt);
    void g2p(float dt);

    // HELPERS
    void addParticle(glm::vec3 pos);
    void updateParticleBuffer();

    // GPU
    void initGPU();

private:
    float RHO = 1.0f;
    float GRAVITY = -9.81f;
    float ALPHA = 0.95f;
    int NUM_VELOCITIES;
    int NUM_GROUP_1D;
    int NUM_GROUP_2D;
    int NUM_GROUP_3D;

     
    GLuint particleSSBO;
    GLuint uSSBO, vSSBO, wSSBO;
    GLuint uMassSSBO, vMassSSBO, wMassSSBO;
    GLuint newuSSBO, newvSSBO, newwSSBO;
    GLuint cellTypeSSBO, divSSBO, pressureSSBO;
    GLuint levelSetSSBO, newLevelSetSSBO, particlesLevelSetSSBO, finalLevelSetSSBO;
    GLuint adSSBO, directionSSBO, dAdSSBO, residualSSBO, paramsSSBO;
    GLuint p2gProg, g2pProg, applyAProg, normalizeProg, classifyCellsProg, resetCellTypesProg, computeDivProg, applyPressureProg, dotProductProg, moveAlphaProg, moveBetaProg, transitionProg, updateLevelSetProg, redistanceProg;
    std::vector<float> Ad, direction, residual;
    SolverParams params;

    // HELPERS
    float dotProduct(const std::vector<float>& a, const std::vector<float>& b);

    // GPU
    template<typename T> void initBuffer(int index, GLuint& ssbo, const std::vector<T>& data);
    template<typename T> void sendDataToGPU(GLuint& ssbo, const std::vector<T>& data);
    template<typename T> void getDataFromGPU(GLuint& ssbo, std::vector<T>& data);
    void dispatchCompute(GLuint prog, int numX=1, int numY=1, int numZ=1);
    void clearBuffer(GLuint buffer);
    void clearBufferInt(GLuint buffer, int value);
};