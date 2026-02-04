#version 450 core

layout(std430, binding = 5) buffer vMassBuffer { float vsMasse[]; };
layout(std430, binding = 10) buffer cellTypeBuffer { int cellTypes[]; };

in vec3 vPos;
uniform int size;

out vec4 FragColor;

float getMass(int i, int j, int k) {
    i = clamp(i, 0, size);
    j = clamp(j, 0, size);
    k = clamp(k, 0, size);
    return vsMasse[k * size * (size + 1) + j * size + i];
}

void main() {
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
    float foamThreshold = 100.0;
    float waterThreshold = 300.0;
    vec3 finalColor;

    if (density < foamThreshold) {
        float foamAlpha = smoothstep(0, foamThreshold, density);
        finalColor = mix(foamColor, waterColor, foamAlpha);
    } else {
        float waterAlpha = smoothstep(foamThreshold, waterThreshold, density);
        finalColor =  mix(waterColor, deepWaterColor, waterAlpha);
    }

    FragColor = vec4(finalColor, 1.0);
}