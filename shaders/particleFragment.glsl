#version 450 core

layout(std430, binding = 5) buffer vMassBuffer { float vsMasse[]; };
layout(std430, binding = 10) buffer cellTypeBuffer { int cellTypes[]; };
// layout(std430, binding = 13) buffer LevelSetBuffer { float levelSet[]; };

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

    


    // vec3 pos = vPos * vec3(float(size));
    // ivec3 iPos = ivec3(floor(pos));
    // iPos = clamp(iPos, ivec3(0), ivec3(size - 1));

    // float l = levelSet[iPos.z * size * size + iPos.y * size + iPos.x];

    // if (l > 3.0f / size) discard;

    // vec3 foamColor = vec3(1.0);
    // vec3 waterColor = vec3(0.1, 0.7, 1.0);
    // vec3 deepWaterColor = vec3(0.0, 0.1, 0.4);

    // float foamThreshold = 0.02;
    // float waterThreshold = 0.05;
    // float deepThreshold = 0.1;
    // vec3 finalColor;
    // if (l < foamThreshold) {
    //     finalColor = foamColor;
    // } else if (l < waterThreshold) {
    //     float t = smoothstep(foamThreshold, waterThreshold, l);
    //     finalColor = mix(foamColor, waterColor, t);
    // } else {
    //     float t = smoothstep(waterThreshold, deepThreshold, l);
    //     finalColor = mix(waterColor, deepWaterColor, t);
    // }

    FragColor = vec4(0.1, 0.7, 1.0, 1.0);
}