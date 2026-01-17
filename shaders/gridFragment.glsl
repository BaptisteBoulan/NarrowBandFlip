#version 450 core

out vec4 FragColor;

in vec2 TexCoord;

uniform sampler2D gridTexture;
uniform float multiplier;

void main() {
    float val = multiplier * texture(gridTexture, TexCoord).r;
    
    vec3 color = vec3(0.0);
    if(val > 0.0) color = vec3(val, 0.0, 0.0); 
    else color = vec3(0.0, 0.0, abs(val));
    
    FragColor = vec4(color + 0.1, 1.0);
    
}