#version 330 core

// Input cube vertices (-1 to +1 unit cube)
layout(location = 0) in vec3 aPos;

uniform mat4 model;       // decal box transform (world space)
uniform mat4 viewProj;    // camera projection * view matrix

// Pass world position to fragment shader
out vec3 FragPosWorld;

void main()
{
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPosWorld = worldPos.xyz;
    gl_Position = viewProj * worldPos;
}