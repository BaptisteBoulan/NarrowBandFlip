#version 450 core

layout(std430, binding = 5) buffer vMassBuffer { float vsMasse[]; };
layout(std430, binding = 10) buffer cellTypeBuffer { int cellTypes[]; };
layout(std430, binding = 13) buffer LevelSetBuffer { float levelSet[]; };

in vec3 vPos;
in float vPointSize;

uniform int size;

out vec4 FragColor;

float getMass(int i, int j, int k) {
    i = clamp(i, 0, size - 1);
    j = clamp(j, 0, size - 1);
    k = clamp(k, 0, size - 1);
    return vsMasse[k * size * size + j * size + i];
}

void main() {
    
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = dot(coord, coord);
    if (dist > 0.25) {
        discard;
    }

    vec3 pos = vPos * vec3(float(size));
    ivec3 iPos = ivec3(floor(pos));
    iPos = clamp(iPos, ivec3(0), ivec3(size - 1));

    float l = levelSet[iPos.z * size * size + iPos.y * size + iPos.x];

    // vec3 waterColor = vec3(0.5, 0.7, 1.0);
    vec3 waterColor = vec3(1.0);
    vec3 deepWaterColor = vec3(0.05, 0.15, 0.3);

    float depthFactor = clamp(l * 10.0, 0.0, 1.0);
    vec3 finalColor = mix(waterColor, deepWaterColor, depthFactor);

    FragColor = vec4(finalColor, 1.0);
}