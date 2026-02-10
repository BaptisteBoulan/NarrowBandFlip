#include "marching_cubes.h"
#include "../noise/perlin.h"


Mesh * generateCubeMesh(std::vector<float> fieldValues, glm::vec3 size, glm::vec3 position) {
    Mesh * mesh = new Mesh();

    std::vector<float> positions, colors;
    std::vector<unsigned int> indices;

    for (auto e: EdgeVertexIndices) {
        int i1 = e[0];
        int i2 = e[1];

        float a = fieldValues[i1];
        float b = fieldValues[i2];

        float t = 0.5f;
        if(a*b < 0.01f)
            t = b / (b-a);

        auto p1 = cubePositions[i1];
        auto p2 = cubePositions[i2];

        float x = t * p1[0] + (1-t) * p2[0];
        float y = t * p1[1] + (1-t) * p2[1];
        float z = t * p1[2] + (1-t) * p2[2];

        positions.push_back(x * size.x + position.x);
        positions.push_back(y * size.y + position.y);
        positions.push_back(z * size.z + position.z);

        colors.push_back(x * size.x + position.x);
        colors.push_back(y * size.y + position.y);
        colors.push_back(z * size.z + position.z);
    }
    int tableIndex = 0;
    int a = 1;
    for (int i = 0; i <8; i++) {
        if (fieldValues[i] > 0.0f) tableIndex += a;
        a *= 2;
    }

    auto& faceIndices = TriangleTable[tableIndex];

    for (int i = 0; faceIndices[i] != -1; ++i) {
        indices.push_back(faceIndices[i]);
    }

    mesh->setPositions(positions);
    mesh->setColors(colors);
    mesh->setIndices(indices);

    return mesh;
}

std::vector<float> generateField(int sizeX, int sizeY, int sizeZ) {
    std::vector<float> field;

    for (int i = 0; i < sizeX; i++) {
        float x = float(i) / sizeX - 0.5f;
        for (int j = 0; j < sizeY; j++) {
            float y = float(j) / sizeY - 0.5f;
            for (int k = 0; k < sizeZ; k++) {
                float z = float(k) / sizeZ - 0.5f;

                field.push_back(perlin3D(glm::vec3(x, y, z)));
            }
        }
    }
    return field;
}

std::vector<Mesh*> generateIsosurface(const std::vector<float>& field, int sizeX, int sizeY, int sizeZ) {
    std::vector<Mesh*> cubes;

    glm::vec3 size = glm::vec3(1.0f / (sizeX-1), 1.0f / (sizeY-1), 1.0f / (sizeZ-1));

    for (int i = 0; i < sizeX - 1; i++) {
        float x = float(i) * size.x;
        for (int j = 0; j < sizeY - 1; j++) {
            float y = float(j) * size.y;
            for (int k = 0; k < sizeZ - 1; k++) {
                float z = float(k) * size.z;

                int index = (i * sizeY + j) * sizeZ + k;

                std::vector<float> fieldCube = {
                    field[index],
                    field[index + sizeY * sizeZ],
                    field[index + sizeZ],
                    field[index + sizeY + sizeY * sizeZ],
                    field[index + 1],
                    field[index + 1 + sizeY * sizeZ],
                    field[index + 1 + sizeZ],
                    field[index + 1 + sizeY + sizeY * sizeZ],
                };

                Mesh * cube = generateCubeMesh(fieldCube, size, glm::vec3(x, y, z));

                if (!cube->getIndices().empty())
                    cubes.push_back(cube);
            }
        }
    }

    return cubes;
}