#version 430 core

layout (location = 0) in vec2 vPos;
layout (location = 1) in vec2 vTextCoords;

out vec2 TexCoords;

void main() {
    TexCoords = vTextCoords;
    
    gl_Position = vec4(vPos.x, vPos.y, 0.0, 1.0);
}