#version 450 core

uniform float color;

out vec4 FragColor;

void main() {
    if (color < 0.5) FragColor = vec4(0.7, 0.9, 1.0, 1.0);
    else FragColor = vec4(0.0, 1.0, 0.0, 1.0);
}