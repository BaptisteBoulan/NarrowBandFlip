#include "Simulation.h"
#include "shader.h"

// Global State
int simRes = 32;
Simulation sim(simRes); 
GLFWwindow* window;
GLuint particleVAO, particleVBO;
GLuint particleShader;
bool paused = true;

// Mouse
bool leftMouseDown = false;
float mouseX, mouseY;
float SPAWN_RATE = 1.0f / 400.0f;


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
        if (key == GLFW_KEY_ENTER) {sim = Simulation(simRes); sim.initGPU();}// Restart simulation
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

    mouseX = xpos / (float)width;
    mouseY = 1.0f - (ypos / (float)height); 
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
}


void initShaders() {
    std::vector<std::pair<char*, ShaderType>> particleShaders = {
        {"shaders/particleVertex.glsl", ShaderType::VERTEX},
        {"shaders/particleFragment.glsl", ShaderType::FRAGMENT},
    };
    particleShader = createShaderProgram(particleShaders);
}


void render() {
    GLuint colorLocation = glGetUniformLocation(particleShader, "color");

    float lastTime = 0;
    float spawnTimer = 0;

    while (!glfwWindowShouldClose(window)) {
        float currentTime = (float)glfwGetTime();
        float dt = std::min(currentTime - lastTime, 0.02f);
        lastTime = currentTime;


        // --- MOUSE SPAWNING LOGIC ---
        if (leftMouseDown && !paused) {
            spawnTimer += dt;
            while (spawnTimer >= SPAWN_RATE) {
                sim.addParticle(glm::vec2((float)mouseX, (float)mouseY));
                spawnTimer -= SPAWN_RATE;
            }
            glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), sim.particles.data(), GL_DYNAMIC_DRAW);
        }
        
        if (!paused) {
            sim.p2g();
            sim.applyForces(dt);
            sim.computeDivergences(dt);
            sim.solvePressure(dt);
            sim.applyPressure(dt);
            sim.g2p(dt);    
        }
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

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
    }
}

int main() {
    initGL();
    initShaders();
    render();
    glfwTerminate();
    return 0;
}