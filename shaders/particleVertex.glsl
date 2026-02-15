#version 450 core

layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 vPos;
out float vPointSize;

void main() {
    vec4 worldPos = model * vec4(aPos, 1.0);
    vec4 viewPos = view * worldPos;
    float distance = -viewPos.z; // Distance from the camera

    // Attenuate the point size based on distance
    float size = 5.0 / distance;
    gl_PointSize = size;
    vPointSize = size;

    gl_Position = projection * viewPos;
    vPos = aPos;
}
