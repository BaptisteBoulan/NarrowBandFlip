#pragma once
#include "config.h"

enum class ShaderType {
    VERTEX,
    FRAGMENT,
    GEOMETRY,
    TESS_CONTROL,
    TESS_EVAL,
    COMPUTE
};

GLenum getShaderType(ShaderType type);
std::string loadShaderSource(const char* path);
GLuint compileShader(GLenum type, const char* path);
GLuint createShaderProgram(const std::vector<std::pair<char*, ShaderType>> shaders);