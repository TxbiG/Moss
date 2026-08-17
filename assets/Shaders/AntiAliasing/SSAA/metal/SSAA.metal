fragment float4 fs_ssaa_downsample(FSIn in [[stage_in]],
                                   texture2d<half> hiResTex [[texture(0)]],
                                   sampler samp [[sampler(0)]],
                                   constant float2& ratio [[buffer(0)]]) // ratio = hiResSize / outSize
{
    // Map output UV to high-res UV space
    float2 srcUV = in.uv * ratio;
    float2 texel = 1.0 / ratio; // in UV units
    // sample 4 neighbouring hi-res samples (simple downsample, adapt for scale)
    float3 c = float3(0.0);
    // sample a 2x2 block centered at srcUV
    c += float3(hiResTex.sample(samp, srcUV + texel * float2(-0.25, -0.25)));
    c += float3(hiResTex.sample(samp, srcUV + texel * float2( 0.25, -0.25)));
    c += float3(hiResTex.sample(samp, srcUV + texel * float2(-0.25,  0.25)));
    c += float3(hiResTex.sample(samp, srcUV + texel * float2( 0.25,  0.25)));
    c *= 0.25;
    return float4(c, 1.0);
}
