#version 330 core

in vec3 FragPosWorld;
uniform sampler2D gAlbedo;
uniform sampler2D gNormal;
uniform sampler2D gDepth;
uniform sampler2D decalTexture;

uniform mat4 invViewProj;
uniform mat4 decalWorldToLocal;

out vec4 FragColor;

void main() {
    vec4 worldPos = vec4(FragPosWorld, 1.0);

    // Transform to decal local space
    vec3 localPos = vec3(decalWorldToLocal * worldPos);

    // Check if inside decal box (-1 to 1 cube)
    if (abs(localPos.x) > 1.0 || abs(localPos.y) > 1.0 || abs(localPos.z) > 1.0)
        discard;

    // Map to [0, 1] UV
    vec2 uv = localPos.xy * 0.5 + 0.5;

    vec4 decalCol = texture(decalTexture, uv);

    // Read base albedo from g-buffer and blend
    vec4 baseCol = texture(gAlbedo, gl_FragCoord.xy / screenSize);
    FragColor = mix(baseCol, decalCol, decalCol.a);
}