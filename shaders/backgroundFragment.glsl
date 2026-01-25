#version 430 core
out vec4 FragColor;

layout(std430, binding = 3) buffer uMassBuffer { float uMasses[]; };
layout(std430, binding = 8) buffer cellTypeBuffer { int cellTypes[]; };

uniform int size;
in vec2 TexCoords;

// Helper to safely fetch mass from the buffer
float getMass(int i, int j) {
    i = clamp(i, 0, size);
    j = clamp(j, 0, size);
    return uMasses[j * (size + 1) + i];
}

void main() {
    // 1. Calculate continuous coordinates in grid space
    vec2 pos = TexCoords * vec2(float(size));

    // 2. Find the top-left integer coordinate and the fractional offset
    ivec2 iPos = ivec2(floor(pos));
    vec2 f = fract(pos);

    // 3. Smooth the fractional weights (Cubic Hermite Spline)
    vec2 w = f * f * (3.0 - 2.0 * f);

    // 4. Sample the 4 nearest neighbors (Bilinear fetch with Cubic weights)
    float m00 = getMass(iPos.x,     iPos.y);
    float m10 = getMass(iPos.x + 1, iPos.y);
    float m01 = getMass(iPos.x,     iPos.y + 1);
    float m11 = getMass(iPos.x + 1, iPos.y + 1);

    // Interpolate along X, then Y
    float massX1 = mix(m00, m10, w.x);
    float massX2 = mix(m01, m11, w.x);
    float smoothedDensity = mix(massX1, massX2, w.y);

    // 5. Coloring Logic
    int type = cellTypes[int(pos.y) * size + int(pos.x)];
    vec3 backgroundColor = vec3(0.1, 0.1, 0.1);
    vec3 foamColor = vec3(1.0);
    vec3 waterColor = vec3(0.1, 0.7, 1.0);
    vec3 deepWaterColor = vec3(0.0, 0.2, 0.5);

    if (type == 0) {
        FragColor = vec4(0.2, 0.2, 0.2, 1.0);
    } else {
        float alpha = clamp(smoothedDensity * 0.5, 0.0, 1.0);
        // Foam effect: if density is low but not zero, use foamColor
        float foamThreshold = 0.1;
        float waterThreshold = 5.0;
        float deepWaterThreshold = 50.0;
        vec3 finalColor;

        if (smoothedDensity < foamThreshold) {
            finalColor = backgroundColor;
        } else if (smoothedDensity < waterThreshold) {
            // Interpolate between foam and water as density increases
            float foamAlpha = smoothstep(foamThreshold, waterThreshold, smoothedDensity);
            finalColor = mix(foamColor, waterColor, foamAlpha);
        } else {
            float waterAlpha = smoothstep(waterThreshold, 50.0, smoothedDensity);
            finalColor =  mix(waterColor, deepWaterColor, waterAlpha);
        }

        FragColor = vec4(finalColor, 1.0);
    }
}
