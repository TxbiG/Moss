#version 450
layout(location=0) in vec2 uv;
layout(location=0) out vec4 outColor;

layout(binding=0) uniform sampler2D uHiRes;
layout(push_constant) uniform PC { vec2 invHiRes; ivec2 scale; } pc;

void main(){
    // Map low-res uv to high-res pixel center
    vec2 hiUV = uv * vec2(pc.scale);
    // compute integer hi-res center in UV space
    ivec2 base = ivec2(floor(hiUV * vec2(1.0) )); // approximate
    vec3 sum = vec3(0.0);
    int cnt = 0;
    for (int y = 0; y < pc.scale.y; ++y) {
        for (int x = 0; x < pc.scale.x; ++x) {
            vec2 sampleUV = (vec2(base + ivec2(x,y)) + 0.5) * pc.invHiRes;
            sum += texture(uHiRes, sampleUV).rgb;
            cnt++;
        }
    }
    outColor = vec4(sum / float(cnt), 1.0);
}
