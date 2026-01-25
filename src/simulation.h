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
    void print() {
        std::cout<<"dAd : "<<dAd<<std::endl;
        std::cout<<"deltaNew : "<<deltaNew<<std::endl;
        std::cout<<"deltaOld : "<<deltaOld<<std::endl;
        std::cout<<"alpha : "<<alpha<<std::endl;
        std::cout<<std::endl;
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
                if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                    grid.cellType[grid.gridIdx(i, j)] = CellType::SOLID;
                } else {
                    grid.cellType[grid.gridIdx(i, j)] = CellType::AIR;
                }
            }
        }

        for (int i = size / 2 - 2; i <= size / 2 + 2; i++) {
            for (int j = size / 4 - 2; j <= size / 4 + 2; j++) {
                grid.cellType[grid.gridIdx(i, j)] = CellType::SOLID;
            }
        }

        // Init particles
        glm::vec2 p1(0.2f, 0.4f);
        glm::vec2 p2(0.8f, 0.8f);
        float spacing = 0.01f;

        for(float x = p1.x; x < p2.x; x += spacing) {
            for (float y = p1.y; y < p2.y; y += spacing) {
                particles.emplace_back(glm::vec2(x,y));
            }
        }

        Ad.assign(grid.total_size, 0.0f);
        direction.assign(grid.total_size, 0.0f);
        residual.assign(grid.total_size, 0.0f);

        params = SolverParams({0,0,0,0});
        NUM_GROUP_1D = (grid.total_size + 255) / 256;
        NUM_GROUP_2D = (size + 16) / 16;
    }

    // STEPS
    void p2g(float dt);
    void computeDivergences(float dt);
    void solvePressure(float dt);
    void applyPressure(float dt);
    void g2p(float dt);

    // HELPERS
    void addParticle(glm::vec2 pos);

    // GPU
    void initGPU();

private:
    float RHO = 1.0f;
    float GRAVITY = -9.81f;
    float ALPHA = 0.95f;
    int NUM_GROUP_1D;
    int NUM_GROUP_2D;

    
    GLuint particleSSBO, uSSBO, vSSBO, uMassSSBO, vMassSSBO, newuSSBO, newvSSBO, pressureSSBO, cellTypeSSBO, adSSBO, directionSSBO, dAdSSBO, divSSBO, residualSSBO, paramsSSBO;
    GLuint p2gProg, g2pProg, applyAProg, normalizeProg, classifyCellsProg, resetCellTypesProg, computeDivProg, applyPressureProg, dotProductProg, moveAlphaProg, moveBetaProg, transitionProg;
    std::vector<float> Ad, direction, residual;
    SolverParams params;

    // HELPERS
    float dotProduct(const std::vector<float>& a, const std::vector<float>& b);
    void updateParticleBuffer();

    // GPU
    template<typename T> void initBuffer(int index, GLuint& ssbo, const std::vector<T>& data);
    template<typename T> void sendDataToGPU(GLuint& ssbo, const std::vector<T>& data);
    template<typename T> void getDataFromGPU(GLuint& ssbo, std::vector<T>& data);
    void dispatchCompute(GLuint prog, int numX=1, int numY=1, int numZ=1);
    void clearBuffer(GLuint buffer);
};