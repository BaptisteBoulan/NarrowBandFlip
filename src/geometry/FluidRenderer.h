#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "table.h"
#include "../config.h"
#include "../shader.h"
#include "../simulation.h"

class FluidRenderer {
public:
    FluidRenderer(int size, Simulation* sim);
    ~FluidRenderer();

    void update();
    void draw(const glm::mat4& view, const glm::mat4& projection, const glm::mat4& model);

private:
    int size;
    GLuint VAO, VBO, EBO;
    GLuint shaderProg;

    Simulation* sim;
    
    std::vector<float> vertices; // pos(3) + normal(3)
    std::vector<unsigned int> indices;

    glm::vec3 getVertex(int edgeIndex, int x, int y, int z);
    glm::vec3 computeNormal(glm::vec3 pos);
    float sampleLevelSet(int x, int y, int z);
    CellType sampleCellType (int x, int y, int z);
    float interpolateLevelSet(glm::vec3 pos);
};
