#include "Simulation.h"
#include "shader.h"

enum class RenderMode { PRESSURE, DIVERGENCE };
RenderMode currentMode = RenderMode::PRESSURE;

// Global State
Simulation sim(64); 
GLFWwindow* window;
GLuint particleVAO, particleVBO;
GLuint gridVAO, gridVBO, textureID;
GLuint particleShader, gridShader;

float quadVertices[] = {
    // Pos      // Tex
    -1.0f,  1.0f, 0.0f, 1.0f,
    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 1.0f, 0.0f,

    -1.0f,  1.0f, 0.0f, 1.0f,
     1.0f, -1.0f, 1.0f, 0.0f,
     1.0f,  1.0f, 1.0f, 1.0f
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_1) currentMode = RenderMode::PRESSURE;
        if (key == GLFW_KEY_2) currentMode = RenderMode::DIVERGENCE;
    }
}

void initGL() {
    glfwInit();
    window = glfwCreateWindow(800, 800, "Fluid Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSetKeyCallback(window, key_callback);

    // --- Grid Setup ---
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    // --- Particle Setup ---
    glGenVertexArrays(1, &particleVAO);
    glGenBuffers(1, &particleVBO);
    glBindVertexArray(particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    // Pre-allocate buffer for maximum particles
    glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)0);
}


void initShaders() {
    std::vector<std::pair<char*, ShaderType>> particleShaders = {
        {"shaders/particleVertex.glsl", ShaderType::VERTEX},
        {"shaders/particleFragment.glsl", ShaderType::FRAGMENT},
    };
    particleShader = createShaderProgram(particleShaders);

    
    std::vector<std::pair<char*, ShaderType>> gridShaders = {
        {"shaders/gridVertex.glsl", ShaderType::VERTEX},
        {"shaders/gridFragment.glsl", ShaderType::FRAGMENT},
    };
    gridShader = createShaderProgram(gridShaders);
}


void render() {
    GLuint multiplierLocation = glGetUniformLocation(gridShader, "multiplier");
    while (!glfwWindowShouldClose(window)) {
        sim.update(0.016f);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        // TEXTURE DISPLAY
        glUseProgram(gridShader);
        const std::vector<float>& gridData = (currentMode == RenderMode::PRESSURE) ? sim.grid.pressure : sim.grid.divergence;

        float multiplier = (currentMode == RenderMode::PRESSURE) ? 3.0f : 1.0f;
        glUniform1f(multiplierLocation, multiplier);

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sim.grid.size, sim.grid.size, 0, GL_RED, GL_FLOAT, gridData.data());
        
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // PARTICLE DISPLAY
        glUseProgram(particleShader);
        glBindVertexArray(particleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
        // Update only the data, don't reallocate the buffer
        glBufferSubData(GL_ARRAY_BUFFER, 0, sim.particles.size() * sizeof(Particle), sim.particles.data());
        glPointSize(3.0f);
        glDrawArrays(GL_POINTS, 0, (GLsizei)sim.particles.size());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}



int main() {
    initGL();
    initShaders();
    render();
    glfwTerminate();
    return 0;
}