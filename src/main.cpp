#include "SimulationNoParticle.h"
// #include "Simulation.h"
#include "shader.h"

enum class RenderMode { PRESSURE, DIVERGENCE };
RenderMode currentMode = RenderMode::PRESSURE;

// Global State
int simRes = 32;
Simulation sim(simRes); 
GLFWwindow* window;
GLuint particleVAO, particleVBO;
GLuint gridVAO, gridVBO, textureID;
GLuint velocityVAO, velocityVBO;
GLuint particleShader, gridShader;
bool paused = true;

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
        if (key == GLFW_KEY_SPACE) paused = !paused;         // Pause/Play simulation
        if (key == GLFW_KEY_ENTER) sim = Simulation(simRes); // Restart simulation
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

    // --- Velocity Field Setup ---
    glGenVertexArrays(1, &velocityVAO);
    glGenBuffers(1, &velocityVBO);
    glBindVertexArray(velocityVAO);
    glBindBuffer(GL_ARRAY_BUFFER, velocityVBO);
    // Pre-allocate: 2 vertices (base/tip) per grid cell
    size_t vertexCount = sim.grid.total_size * 2;
    glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(glm::vec2), NULL, GL_DYNAMIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
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
    GLuint colorLocation = glGetUniformLocation(particleShader, "color");

    float lastTime = 0;

    while (!glfwWindowShouldClose(window)) {
        float currentTime = glfwGetTime();
        float dt = currentTime - lastTime;
        lastTime = currentTime;
        
        if (!paused) sim.update(0.1f);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        // TEXTURE DISPLAY
        glUseProgram(gridShader);
        const std::vector<float>& gridData = (currentMode == RenderMode::PRESSURE) ? sim.grid.pressure : sim.grid.divergence;

        float multiplier = (currentMode == RenderMode::PRESSURE) ? 0.5f : -0.01f;
        glUniform1f(multiplierLocation, multiplier);

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sim.grid.size, sim.grid.size, 0, GL_RED, GL_FLOAT, gridData.data());
        
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // VELOCITY DISPLAY
        std::vector<glm::vec2> velocityLines;
        for (int y = 0; y < sim.size; ++y) {
            for (int x = 0; x < sim.size; ++x) {
                int idx = sim.grid.gridIdx(x,y);
                glm::vec2 vel = sim.grid.interpolatedVelocities[idx];

                float posX = (x+0.5f) * sim.h;
                float posY = (y+0.5f) * sim.h;
                glm::vec2 basePos(posX, posY);
                
                velocityLines.push_back(basePos);
                velocityLines.push_back(basePos + vel * sim.h);
            }
        }
        glUseProgram(particleShader);
        glBindVertexArray(velocityVAO);
        glBindBuffer(GL_ARRAY_BUFFER, velocityVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, velocityLines.size() * sizeof(glm::vec2), velocityLines.data());
        glUniform1f(colorLocation, 1.0f);
        glDrawArrays(GL_LINES, 0, (GLsizei)velocityLines.size());


        // PARTICLE DISPLAY
        glUseProgram(particleShader);
        glBindVertexArray(particleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
        // Update only the data, don't reallocate the buffer
        glBufferSubData(GL_ARRAY_BUFFER, 0, sim.particles.size() * sizeof(Particle), sim.particles.data());
        glPointSize(5.0f);
        glUniform1f(colorLocation, 0.0f);
        glDrawArrays(GL_POINTS, 0, (GLsizei)sim.particles.size());

        glfwSwapBuffers(window);
        glfwPollEvents();
        glfwSwapInterval(1);
    }
}



int main() {
    initGL();
    initShaders();
    render();
    glfwTerminate();
    return 0;
}