cbuffer CB : register(b0) { float2 InvDstTexSize; float2 Scale; /* Scale = hi / dst */ };

Texture2D Hi : register(t0);
SamplerState LinearClamp : register(s0);

float4 PS_Downsample(float4 pos : SV_POSITION, float2 uv : TEXCOORD0) : SV_TARGET
{
    // Simple 4x box filter for 2x SSAA (scales generalize)
    float2 hiUV = uv * Scale;
    float2 texel = 1.0 / Scale; // texel in hi UV space
    // For 2x: average 4 samples
    float3 c = float3(0,0,0);
    c += Hi.Sample(LinearClamp, hiUV + texel * float2(-0.25, -0.25)).rgb;
    c += Hi.Sample(LinearClamp, hiUV + texel * float2( 0.25, -0.25)).rgb;
    c += Hi.Sample(LinearClamp, hiUV + texel * float2(-0.25,  0.25)).rgb;
    c += Hi.Sample(LinearClamp, hiUV + texel * float2( 0.25,  0.25)).rgb;
    c *= 0.25;
    return float4(c, 1.0);
}
