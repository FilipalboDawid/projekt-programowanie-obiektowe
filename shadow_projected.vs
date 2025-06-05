#version 330

in vec3 vertexPosition;

uniform mat4 matModel;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 shadowMat;

void main()
{
    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    vec4 shadowPos = shadowMat * worldPos;
    gl_Position = projection * view * shadowPos;
}
