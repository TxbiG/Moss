#version 450
layout(location=0) in vec2 uv;
layout(location=0) out vec4 outColor;

layout(binding=0) uniform sampler2D uScene;
layout(push_constant) uniform PC { vec2 invTexSize; float edgeThreshold; float edgeThresholdMin; } pc;

float luma(vec3 c){ return dot(c, vec3(0.299,0.587,0.114)); }

void main(){
    vec3 rgbM = texture(uScene, uv).rgb;
    float lM = luma(rgbM);

    vec3 rgbN = texture(uScene, uv + vec2(0.0, -pc.invTexSize.y)).rgb;
    vec3 rgbS = texture(uScene, uv + vec2(0.0,  pc.invTexSize.y)).rgb;
    vec3 rgbW = texture(uScene, uv + vec2(-pc.invTexSize.x, 0.0)).rgb;
    vec3 rgbE = texture(uScene, uv + vec2( pc.invTexSize.x, 0.0)).rgb;

    float lN = luma(rgbN), lS = luma(rgbS), lW = luma(rgbW), lE = luma(rgbE);

    float lMin = min(lM, min(min(lN, lS), min(lW, lE)));
    float lMax = max(lM, max(max(lN, lS), max(lW, lE)));

    if (lMax - lMin < max(pc.edgeThresholdMin, lMax * pc.edgeThreshold)) {
        outColor = vec4(rgbM, 1.0);
        return;
    }

    // approximate gradient direction
    vec2 dir;
    dir.x = -((lN + lS) - 2.0*lM);
    dir.y =  ((lW + lE) - 2.0*lM);

    float len = max(abs(dir.x), abs(dir.y));
    if (len < 1e-5) { outColor = vec4(rgbM, 1.0); return; }
    dir = normalize(dir) * pc.invTexSize;

    // sample along edge
    vec3 rgbA = texture(uScene, uv + dir * (1.0/3.0 - 0.5)).rgb;
    vec3 rgbB = texture(uScene, uv + dir * (2.0/3.0 - 0.5)).rgb;
    vec3 blended = (rgbA + rgbB) * 0.5;

    outColor = vec4(blended, 1.0);
}
