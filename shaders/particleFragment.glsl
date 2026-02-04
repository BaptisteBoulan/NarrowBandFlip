#version 450 core

in vec3 vPos;

out vec4 FragColor;

void main() {
    FragColor = vec4(5.0f * vPos.y, 0.0, 1.0, 1.0);
}