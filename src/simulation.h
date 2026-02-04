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
                    if (i == 0 || i == size - 1 || j == 0 || j == size - 1 || k == -1 || k == size - 1) {
                        grid.cellType[grid.gridIdx(i, j, k)] = CellType::SOLID;
                    } else {
                        grid.cellType[grid.gridIdx(i, j, k)] = CellType::AIR;
                    }
                }
            }
        }
        // Init particles
        glm::vec3 p1(0.6f, 0.05f, 0.01f);
        glm::vec3 p2(0.95f, 0.95f, 0.3f);
        float spacing = 0.5f / size;

        for(float x = p1.x; x < p2.x; x += spacing) {
            for (float y = p1.y; y < p2.y; y += spacing) {
                for (float z = p1.z; z < p2.z; z += spacing)
                    particles.emplace_back(glm::vec3(x,y,z));
            }
        }

        Ad.assign(grid.total_cells, 0.0f);
        direction.assign(grid.total_cells, 0.0f);
        residual.assign(grid.total_cells, 0.0f);

        params = SolverParams({0,0,0,0});
        NUM_GROUP_1D = (grid.total_cells + 255) / 256;
        NUM_GROUP_2D = (size + 16) / 16;
        NUM_GROUP_3D = (size + 8) / 8;

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
    int NUM_GROUP_1D;
    int NUM_GROUP_2D;
    int NUM_GROUP_3D;

     
    GLuint particleSSBO;
    GLuint uSSBO, vSSBO, wSSBO;
    GLuint uMassSSBO, vMassSSBO, wMassSSBO;
    GLuint newuSSBO, newvSSBO, newwSSBO;
    GLuint pressureSSBO, cellTypeSSBO, adSSBO, directionSSBO, dAdSSBO, divSSBO, residualSSBO, paramsSSBO;
    GLuint p2gProg, g2pProg, applyAProg, normalizeProg, classifyCellsProg, resetCellTypesProg, computeDivProg, applyPressureProg, dotProductProg, moveAlphaProg, moveBetaProg, transitionProg;
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
};