#pragma once
#include "../utils/config.h"
#include "mesh.h"
#include "table.h"

std::vector<float> generateField(int sizeX, int sizeY, int sizeZ);

Mesh * generateCubeMesh(std::vector<float> fieldValues, glm::vec3 size, glm::vec3 position);

std::vector<Mesh*> generateIsosurface(const std::vector<float>& field, int sizeX, int sizeY, int sizeZ);