#version 330

in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

out vec4 finalColor;

uniform vec3 lightDirection;
uniform vec4 colDiffuse;

void main() {
    vec3 normal = normalize(fragNormal);
    float light = max(dot(-lightDirection, normal), 0.0);
    vec3 baseColor = vec3(0.8, 0.2, 0.2);
    vec3 color = baseColor * light;
    finalColor = vec4(1.0, 0.0, 0.0, 1.0);
}