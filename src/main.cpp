#include "Simulation.h"
#include "shader.h"

// Global State
int simRes = 64;
Simulation sim(simRes);
GLFWwindow* window;
GLuint particleVAO, particleVBO;
GLuint quadVAO, quadVBO;
GLuint particleShader, backgroundShader;
bool paused = true;
bool showBackground = false;
bool showParticles = true;

// Mouse
bool leftMouseDown = false;
float mouseX, mouseY;
float SPAWN_RATE = 0.1f / simRes;

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
        if (key == GLFW_KEY_SPACE) paused = !paused;         // Pause/Play simulation
        if (key == GLFW_KEY_ENTER) { sim = Simulation(simRes); sim.initGPU(); } // Restart simulation
        if (key == GLFW_KEY_1) showBackground = !showBackground;  // Toggle background
        if (key == GLFW_KEY_2) showParticles = !showParticles;    // Toggle particles
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) leftMouseDown = true;
        else if (action == GLFW_RELEASE) leftMouseDown = false;
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    mouseX = (float)xpos / width;
    mouseY = 1.0f - ((float)ypos / height);
}

void initBackground() {
    float quadVertices[] = {
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    glGenVertexArrays(1, &quadVAO);

    glGenBuffers(1, &quadVBO);

    glBindVertexArray(quadVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
}

void initGL() {
    glfwInit();
    window = glfwCreateWindow(800, 800, "Fluid Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glfwSetKeyCallback(window, key_callback);

    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    // --- Particle Setup ---
    glGenVertexArrays(1, &particleVAO);
    glGenBuffers(1, &particleVBO);
    glBindVertexArray(particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    // Pre-allocate buffer for maximum particles
    glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)0);

    sim.initGPU();
    initBackground();
}

void initShaders() {
    std::vector<std::pair<char*, ShaderType>> particleShaders = {
        {"shaders/particleVertex.glsl", ShaderType::VERTEX},
        {"shaders/particleFragment.glsl", ShaderType::FRAGMENT},
    };
    particleShader = createShaderProgram(particleShaders);

    backgroundShader = createShaderProgram({
        {"shaders/backgroundVertex.glsl", ShaderType::VERTEX},
        {"shaders/backgroundFragment.glsl", ShaderType::FRAGMENT}
    });
}

void render() {
    float lastTime = (float)glfwGetTime();
    float spawnTimer = 0;
    float fpsTimer = 0;
    int frameCount = 0;
    int fps = 0;

    while (!glfwWindowShouldClose(window)) {
        float currentTime = (float)glfwGetTime();
        float dt = currentTime - lastTime;
        lastTime = currentTime;
        frameCount++;

        fpsTimer += dt;
        if (fpsTimer >= 1.0f) {
            fps = frameCount;
            frameCount = 0;
            fpsTimer = 0;
            std::cout << "FPS: " << fps << std::endl;
        }

        // --- MOUSE SPAWNING LOGIC ---
        if (leftMouseDown && !paused) {
            spawnTimer += dt;
            while (spawnTimer >= SPAWN_RATE) {
                sim.addParticle(glm::vec2((float)mouseX, (float)mouseY));
                spawnTimer -= SPAWN_RATE;
            }
            sim.updateParticleBuffer();
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), sim.particles.data(), GL_DYNAMIC_DRAW);
        }

        dt = std::min(dt, 0.02f);

        if (!paused) {
            sim.p2g(dt);
            sim.computeDivergences(dt);
            sim.solvePressure(dt);
            sim.applyPressure(dt);
            sim.g2p(dt);
        }
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // BACKGROUND DISPLAY
        if (showBackground) {
            glUseProgram(backgroundShader);
            glUniform1i(glGetUniformLocation(backgroundShader, "size"), simRes);
            glBindVertexArray(quadVAO);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        // PARTICLE DISPLAY
        if (showParticles) {
            glUseProgram(particleShader);
            glBindVertexArray(particleVAO);
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sim.particles.size() * sizeof(Particle), sim.particles.data());
            glPointSize(1.0f);
            glDrawArrays(GL_POINTS, 0, (GLsizei)sim.particles.size());
        }

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
