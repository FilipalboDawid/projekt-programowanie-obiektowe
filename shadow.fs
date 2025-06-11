#version 330

in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

out vec4 finalColor;

uniform vec3 lightDirection;

void main() {
    finalColor = vec4(1.0, 0.0, 0.0, 1.0);
}