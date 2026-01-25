#version 430 core

struct SolverParams {
    float dAd;
    float deltaNew;
    float deltaOld;
    float alpha;
};

layout(local_size_x = 256) in;

layout(std430, binding = 9) coherent buffer AdBuffer { float Ad[]; };
layout(std430, binding = 10) coherent buffer DirectionBuffer { float direction[]; };
layout(std430, binding = 12) coherent buffer ResidualBuffer { float residual[]; };
layout(std430, binding = 13) coherent buffer ParamsBufferUint {
    uint dAd;
    uint deltaNew;
    uint deltaOld;
    uint alpha;
} paramsUint;


uniform int size;
uniform int computeDelta;

shared float sharedData[256];

void main() {
    uint gIdx = gl_GlobalInvocationID.x;
    uint lIdx = gl_LocalInvocationIndex;
    int totalElements = size * size;

    float val = 0.0;
    if (gIdx < totalElements) {
        if (computeDelta == 1)
            val = residual[gIdx] * residual[gIdx];
        else
            val = Ad[gIdx] * direction[gIdx];

    }
    sharedData[lIdx] = val;
    barrier();

    for (uint stride = gl_WorkGroupSize.x / 2; stride > 0; stride >>= 1) {
        if (lIdx < stride) {
            sharedData[lIdx] += sharedData[lIdx + stride];
        }
        barrier();
    }

    if (lIdx == 0) {
        float workgroupSum = sharedData[0];
        uint expected, current, next;
        
        current = (computeDelta == 1) ? paramsUint.deltaNew : paramsUint.dAd;
        do {
            expected = current;
            float total = uintBitsToFloat(expected) + workgroupSum;
            next = floatBitsToUint(total);
            if (computeDelta == 1)  current = atomicCompSwap(paramsUint.deltaNew, expected, next);
            else                    current = atomicCompSwap(paramsUint.dAd, expected, next);
        } while (current != expected);
    }
}