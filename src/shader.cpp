#include "shader.h"


GLenum getShaderType(ShaderType type) {
    switch (type)
    {
    case ShaderType::VERTEX:
        return GL_VERTEX_SHADER;

    case ShaderType::FRAGMENT:
        return GL_FRAGMENT_SHADER;

    case ShaderType::GEOMETRY:
        return GL_GEOMETRY_SHADER;

    case ShaderType::TESS_CONTROL:
        return GL_TESS_CONTROL_SHADER;

    case ShaderType::TESS_EVAL:
        return GL_TESS_EVALUATION_SHADER;

    default:
        return GL_COMPUTE_SHADER;
    }
}

std::string loadShaderSource(const char* path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

GLuint compileShader(GLenum type, const char* path) {
    std::string code = loadShaderSource(path);
    const char* src = code.c_str();

    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    // Just to check for errors
    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char errorLog[1024]; //size of the error log
        glGetShaderInfoLog(shader, 1024, NULL, errorLog);
        std::cout<<"Shader Module error"<< errorLog << std::endl;
    }

    return shader;
}

GLuint createShaderProgram(const std::vector<std::pair<char*, ShaderType>> shaders) {
    GLuint program = glCreateProgram();

    for (auto [path, type] : shaders) {
        GLuint shader = compileShader(getShaderType(type), path);
        glAttachShader(program, shader);
        glDeleteShader(shader);
    }

    glLinkProgram(program);

    // Just to check for errors
    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char errorLog[1024]; //size of the error log
        glGetProgramInfoLog(program, 1024, NULL, errorLog);
        std::cout<<"Shader linking error: "<< errorLog << std::endl;
    }

    return program;
}