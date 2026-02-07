#version 450 core

layout(std430, binding = 5) buffer vMassBuffer { float vsMasse[]; };
layout(std430, binding = 10) buffer cellTypeBuffer { int cellTypes[]; };
layout(std430, binding = 13) buffer LevelSetBuffer { int levelSet[]; };

in vec3 vPos;
in float vPointSize;

uniform int size;

out vec4 FragColor;

float getMass(int i, int j, int k) {
    i = clamp(i, 0, size);
    j = clamp(j, 0, size);
    k = clamp(k, 0, size);
    return vsMasse[k * size * (size + 1) + j * size + i];
}

void main() {
    
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = dot(coord, coord);
    if (dist > 0.25) {
        discard;
    }

    vec3 pos = vPos * vec3(float(size));
    ivec3 iPos = ivec3(floor(pos));
    vec3 f = fract(pos);

    vec3 w = f * f * (3.0 - 2.0 * f);

    float m_top = getMass(iPos.x, iPos.y + 1, iPos.z);
    float m_bot = getMass(iPos.x, iPos.y,     iPos.z);

    float density = mix(m_bot, m_top, w.y);

    vec3 foamColor = vec3(1.0);
    vec3 waterColor = vec3(0.1, 0.7, 1.0);
    vec3 deepWaterColor = vec3(0.0, 0.2, 0.5);

    float alpha = clamp(density * 0.5, 0.0, 1.0);
    float foamThreshold = 10.0;
    float waterThreshold = 60.0;
    vec3 finalColor;

    if (density < foamThreshold) {
        float foamAlpha = smoothstep(0, foamThreshold, density);
        finalColor = mix(foamColor, waterColor, foamAlpha);
    } else {
        float waterAlpha = smoothstep(foamThreshold, waterThreshold, density);
        finalColor =  mix(waterColor, deepWaterColor, waterAlpha);
    }

    int l = levelSet[iPos.z * size * size + iPos.y * size + iPos.x];

    // if (l < 0.0f && l > -0.8f / size) finalColor.r = 1.0f;
    // else finalColor.r = 0.0f;

    finalColor.r =  l * 0.0000000003f;
    finalColor.b = -l * 0.0000000003f;
    finalColor.g =  0.0f;


    FragColor = vec4(finalColor, 1.0);
}