// FXAA - minimal
cbuffer CB : register(b0)
{
    float2 InvTexSize; // 1/width, 1/height
    float _pad0;
    float _pad1;
};

Texture2D Src : register(t0);
SamplerState LinearClamp : register(s0);

float luma(float3 c) { return dot(c, float3(0.299, 0.587, 0.114)); }

struct PSIn { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };

float4 PS_FXAA(PSIn IN) : SV_TARGET
{
    float2 uv = IN.uv;
    float3 rgbM = Src.Sample(LinearClamp, uv).rgb;
    float lM = luma(rgbM);

    float3 rgbN = Src.Sample(LinearClamp, uv + float2(0, -InvTexSize.y)).rgb;
    float3 rgbS = Src.Sample(LinearClamp, uv + float2(0,  InvTexSize.y)).rgb;
    float3 rgbW = Src.Sample(LinearClamp, uv + float2(-InvTexSize.x, 0)).rgb;
    float3 rgbE = Src.Sample(LinearClamp, uv + float2( InvTexSize.x, 0)).rgb;

    float lN = luma(rgbN), lS = luma(rgbS), lW = luma(rgbW), lE = luma(rgbE);
    float lMin = min(lM, min(min(lN,lS), min(lW,lE)));
    float lMax = max(lM, max(max(lN,lS), max(lW,lE)));
    float contrast = lMax - lMin;

    if (contrast < 1.0/8.0) { return float4(rgbM,1); }

    float2 dir = float2(-(lN + lS - 2.0*lM), (lW + lE - 2.0*lM));
    float rcp = 1.0/(min(abs(dir.x), abs(dir.y)) + 1e-6);
    dir = clamp(dir * rcp, float2(-8, -8), float2(8, 8)) * InvTexSize;

    float3 c1 = Src.Sample(LinearClamp, uv + dir * (1.0/3.0 - 0.5)).rgb;
    float3 c2 = Src.Sample(LinearClamp, uv + dir * (2.0/3.0 - 0.5)).rgb;
    float3 blended = 0.5 * (c1 + c2);

    return float4(blended, 1.0);
}
