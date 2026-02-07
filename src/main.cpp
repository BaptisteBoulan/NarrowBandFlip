#include "Simulation.h"
#include "shader.h"
#include "camera.h"
#include "frameRecording/FrameRecorder.h"

// Global State
int simRes = 39;
Simulation sim(simRes);
GLFWwindow* window;
GLuint particleVAO, particleVBO;
GLuint particleShader;
bool paused = true;
FrameRecorder frameRecorder; 

// Mouse
bool leftMouseDown = false;
float mouseX, mouseY;
float SPAWN_RATE = 0.03f / simRes;

// Camera
Camera camera(glm::vec3(-0.5f, 0.5f, 1.5f)); 

// Debug
std::vector<float> debugParticles;
GLuint debugParticleVAO, debugParticleVBO;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_P) {
            paused = !paused; 
            if (!frameRecorder.isRecording()) {
                frameRecorder.startRecording();
            } else {
                frameRecorder.stopRecording();
            }
        } // Pause/Play simulation and Start/Stop the recording
        if (key == GLFW_KEY_ENTER) { sim = Simulation(simRes); sim.initGPU(); } // Restart simulation
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

void initGL() {
    glfwInit();
    window = glfwCreateWindow(800, 800, "Fluid Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glfwSetKeyCallback(window, key_callback);

    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);

    // --- Particle Setup ---
    glGenVertexArrays(1, &particleVAO);
    glGenBuffers(1, &particleVBO);
    glBindVertexArray(particleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
    // Pre-allocate buffer for maximum particles
    glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Particle), (void*)0);

    // Debug
    glGenVertexArrays(1, &debugParticleVAO);
    glGenBuffers(1, &debugParticleVBO);

    glBindVertexArray(debugParticleVAO);
    glBindBuffer(GL_ARRAY_BUFFER, debugParticleVBO);
    glBufferData(GL_ARRAY_BUFFER, debugParticles.size() * 3 * sizeof(float), debugParticles.data(), GL_STATIC_READ);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);

    sim.initGPU();
}

void initDebug() {
    for (int i = 0; i < simRes; i += 1) {
        float x = (0.5f + i)/simRes;


        for (int j = 0; j < simRes; j += 1) {
            float y = (0.5f + j)/simRes;

            for (int k = 0; k < simRes; k += 1) {
                float z = (0.5f + k)/simRes;

                debugParticles.push_back(x);
                debugParticles.push_back(y);
                debugParticles.push_back(z);

            }
        }
    }
}

void initShaders() {
    std::vector<std::pair<char*, ShaderType>> particleShaders = {
        {"shaders/particleVertex.glsl", ShaderType::VERTEX},
        {"shaders/particleFragment.glsl", ShaderType::FRAGMENT},
    };
    particleShader = createShaderProgram(particleShaders);
}

void render(bool record = false) {
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
                sim.addParticle(glm::vec3((float)mouseX, 0.8f, (float)mouseY));
                spawnTimer -= SPAWN_RATE;
            }
            sim.updateParticleBuffer();
            glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
            glBufferData(GL_ARRAY_BUFFER, sim.particles.size() * sizeof(Particle), sim.particles.data(), GL_DYNAMIC_DRAW);
        }

        // dt = std::min(dt, 0.02f);
        dt = 0.02f;

        if (!paused) {
            sim.p2g(dt);
            sim.computeDivergences(dt);
            sim.solvePressure(dt);
            sim.applyPressure(dt);
            sim.g2p(dt);
        }

        // CAMERA MOVEMENTS
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, dt);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, dt);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, dt);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, dt);
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera.ProcessKeyboard(UP, dt);
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            camera.ProcessKeyboard(DOWN, dt);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // PARTICLE DISPLAY
        glUseProgram(particleShader);

        // Uniforms
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), 800.0f / 800.0f, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 model = glm::mat4(1.0f);

        glUniformMatrix4fv(glGetUniformLocation(particleShader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(particleShader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(particleShader, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniform1i(glGetUniformLocation(particleShader, "size"), sim.size);
        
        glBindVertexArray(particleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, particleVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sim.particles.size() * sizeof(Particle), sim.particles.data());

        glPointSize(3.0f);
        glDrawArrays(GL_POINTS, 0, (GLsizei)sim.particles.size());

        // glBindVertexArray(debugParticleVAO);
        // glPointSize(3.0f);
        // glDrawArrays(GL_POINTS, 0, (GLsizei)debugParticles.size());


        if (record && frameRecorder.isRecording()) {
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);
            frameRecorder.saveFrame(width, height);
        }


        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

int main() {
    initDebug();
    initGL();
    initShaders();
    render();
    glfwTerminate();
    return 0;
}
