#include <metal_stdlib>
using namespace metal;

fragment float4 fs_fxaa(FSIn in [[stage_in]],
                        texture2d<half, access::sample> srcTex [[texture(0)]],
                        sampler              samp   [[sampler(0)]],
                        constant float2&     invTexSize [[buffer(0)]]) {
    constexpr float3 lumaCoeff = float3(0.299, 0.587, 0.114);
    auto sample = [&](float2 uv){ return float3(srcTex.sample(samp, uv)); };

    float2 uv = in.uv;
    float3 cM = sample(uv);
    float lM = dot(cM, lumaCoeff);

    float3 cN = sample(uv + float2(0.0, -invTexSize.y));
    float3 cS = sample(uv + float2(0.0,  invTexSize.y));
    float3 cW = sample(uv + float2(-invTexSize.x, 0.0));
    float3 cE = sample(uv + float2( invTexSize.x, 0.0));

    float lN = dot(cN, lumaCoeff);
    float lS = dot(cS, lumaCoeff);
    float lW = dot(cW, lumaCoeff);
    float lE = dot(cE, lumaCoeff);

    float lMin = min(lM, min(min(lN, lS), min(lW, lE)));
    float lMax = max(lM, max(max(lN, lS), max(lW, lE)));
    float contrast = lMax - lMin;

    if (contrast < (1.0f / 8.0f)) {
        return float4(cM, 1.0);
    }

    float2 dir = float2(-(lN + lS - 2.0*lM),
                        (lW + lE - 2.0*lM));
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + 1e-6);
    dir = clamp(dir * rcpDirMin, float2(-8.0), float2(8.0)) * invTexSize;

    float3 c1 = sample(uv + dir * (1.0/3.0 - 0.5));
    float3 c2 = sample(uv + dir * (2.0/3.0 - 0.5));
    float3 blended = (c1 + c2) * 0.5;

    return float4(blended, 1.0);
}
