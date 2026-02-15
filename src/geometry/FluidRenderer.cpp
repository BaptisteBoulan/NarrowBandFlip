#include "FluidRenderer.h"
#include "../config.h"

FluidRenderer::FluidRenderer(int size, Simulation* sim) : size(size), sim(sim) {
    std::vector<std::pair<char*, ShaderType>> shaderFiles = {
        {"shaders/rasterVertex.glsl", ShaderType::VERTEX},
        {"shaders/rasterFragment.glsl", ShaderType::FRAGMENT}
    };
    this->shaderProg = createShaderProgram(shaderFiles); // Use your existing helper

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    // Stride will be 6 floats: 3 Pos, 3 Normal
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

FluidRenderer::~FluidRenderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteProgram(shaderProg);
}

float FluidRenderer::sampleLevelSet(int x, int y, int z) {
    if (x < 0 || x >= size || y < 0 || y >= size || z < 0 || z >= size) return 100.0f; // Outside
    if (sim->grid.levelSet[z * size * size + y * size + x] > 4.0f / size) return 1.0f;
    int level = sim->density[z * size * size + y * size + x];
    return level / 1000000.0f;
}

float FluidRenderer::interpolateLevelSet(glm::vec3 pos) {
    // Trilinear interpolation for normals
    float px = pos.x * size - 0.5f;
    float py = pos.y * size - 0.5f;
    float pz = pos.z * size - 0.5f;
    
    int i = (int)floor(px);
    int j = (int)floor(py);
    int k = (int)floor(pz);
    
    float tx = px - i;
    float ty = py - j;
    float tz = pz - k;
    
    float result = 0.0f;
    for (int kk = 0; kk <= 1; ++kk) {
        for (int jj = 0; jj <= 1; ++jj) {
            for (int ii = 0; ii <= 1; ++ii) {
                float val = sampleLevelSet(i + ii, j + jj, k + kk);
                float weight = (ii == 0 ? (1.0f - tx) : tx) *
                               (jj == 0 ? (1.0f - ty) : ty) *
                               (kk == 0 ? (1.0f - tz) : tz);
                result += val * weight;
            }
        }
    }
    return result;
}

glm::vec3 FluidRenderer::computeNormal(glm::vec3 pos) {
    float eps = 1.0f / size;
    float dx = interpolateLevelSet(pos + glm::vec3(eps, 0, 0)) - interpolateLevelSet(pos - glm::vec3(eps, 0, 0));
    float dy = interpolateLevelSet(pos + glm::vec3(0, eps, 0)) - interpolateLevelSet(pos - glm::vec3(0, eps, 0));
    float dz = interpolateLevelSet(pos + glm::vec3(0, 0, eps)) - interpolateLevelSet(pos - glm::vec3(0, 0, eps));
    return -glm::normalize(glm::vec3(dx, dy, dz));
}

glm::vec3 FluidRenderer::getVertex(int edgeIndex, int x, int y, int z) {
    int v1 = EdgeVertexIndices[edgeIndex][0];
    int v2 = EdgeVertexIndices[edgeIndex][1];

    glm::vec3 p1(x + cubePositions[v1][0], y + cubePositions[v1][1], z + cubePositions[v1][2]);
    glm::vec3 p2(x + cubePositions[v2][0], y + cubePositions[v2][1], z + cubePositions[v2][2]);

    float val1 = sampleLevelSet((int)p1.x, (int)p1.y, (int)p1.z);
    float val2 = sampleLevelSet((int)p2.x, (int)p2.y, (int)p2.z);

    float t = 0.5f;
    if (std::abs(val1 - val2) > 1e-5) {
        t = val1 / (val1 - val2);
        t = std::min(1.0f, std::max(0.0f, t));
    }
    
    glm::vec3 interpPos = p1 + t * (p2 - p1);
    return interpPos / (float)size; // Normalize to [0,1]
}

void FluidRenderer::update() {

    vertices.clear();
    indices.clear();
    
    int indexCounter = 0;

    float threshold = 0.1f;

    for (int k = 0; k < size - 1; ++k) {
        for (int j = 0; j < size - 1; ++j) {
            for (int i = 0; i < size - 1; ++i) {

                
                int cubeIndex = 0;
                if (sampleLevelSet(i + 0, j + 0, k + 0) < threshold) cubeIndex |= 1;
                if (sampleLevelSet(i + 1, j + 0, k + 0) < threshold) cubeIndex |= 2;
                if (sampleLevelSet(i + 0, j + 1, k + 0) < threshold) cubeIndex |= 4;
                if (sampleLevelSet(i + 1, j + 1, k + 0) < threshold) cubeIndex |= 8;
                if (sampleLevelSet(i + 0, j + 0, k + 1) < threshold) cubeIndex |= 16;
                if (sampleLevelSet(i + 1, j + 0, k + 1) < threshold) cubeIndex |= 32;
                if (sampleLevelSet(i + 0, j + 1, k + 1) < threshold) cubeIndex |= 64;
                if (sampleLevelSet(i + 1, j + 1, k + 1) < threshold) cubeIndex |= 128; 


                if (TriangleTable[cubeIndex][0] == -1) continue;

                for (int m = 0; TriangleTable[cubeIndex][m] != -1; m += 3) {
                    int edge1 = TriangleTable[cubeIndex][m];
                    int edge2 = TriangleTable[cubeIndex][m + 1];
                    int edge3 = TriangleTable[cubeIndex][m + 2];

                    glm::vec3 p1 = getVertex(edge1, i, j, k);
                    glm::vec3 p2 = getVertex(edge2, i, j, k);
                    glm::vec3 p3 = getVertex(edge3, i, j, k);

                    glm::vec3 n1 = computeNormal(p1);
                    glm::vec3 n2 = computeNormal(p2);
                    glm::vec3 n3 = computeNormal(p3);

                    vertices.push_back(p1.x); vertices.push_back(p1.y); vertices.push_back(p1.z);
                    vertices.push_back(n1.x); vertices.push_back(n1.y); vertices.push_back(n1.z);

                    vertices.push_back(p2.x); vertices.push_back(p2.y); vertices.push_back(p2.z);
                    vertices.push_back(n2.x); vertices.push_back(n2.y); vertices.push_back(n2.z);

                    vertices.push_back(p3.x); vertices.push_back(p3.y); vertices.push_back(p3.z);
                    vertices.push_back(n3.x); vertices.push_back(n3.y); vertices.push_back(n3.z);
                    
                    indices.push_back(indexCounter++);
                    indices.push_back(indexCounter++);
                    indices.push_back(indexCounter++);
                }
            }
        }
    }
    
    glBindVertexArray(VAO);
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_DYNAMIC_DRAW);
}

void FluidRenderer::draw(const glm::mat4& view, const glm::mat4& projection, const glm::mat4& model) {
    glUseProgram(shaderProg);
    
    glUniformMatrix4fv(glGetUniformLocation(shaderProg, "view"), 1, GL_FALSE, &view[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shaderProg, "projection"), 1, GL_FALSE, &projection[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shaderProg, "model"), 1, GL_FALSE, &model[0][0]);
    
    glUniform3f(glGetUniformLocation(shaderProg, "lightPos"), 2.0f, 4.0f, 3.0f);
    glUniform3f(glGetUniformLocation(shaderProg, "viewPos"), 0.0f, 0.0f, 5.0f); // Approximate view pos
    glUniform3f(glGetUniformLocation(shaderProg, "objectColor"), 0.0f, 0.5f, 0.8f); // Fluid color (Blueish)

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
}
