#version 330 core

in vec3 fvertex;
in vec3 fnormal;

uniform vec3  matdiff;
uniform vec3  matspec;
uniform float matshin;
uniform float alpha = 1.0;

// Lighting
const vec3 ambientLight = vec3(0.2, 0.2, 0.2);
const int MAX_LIGHTS = 4;
uniform vec3 lightPos[MAX_LIGHTS];
uniform vec3 lightColor[MAX_LIGHTS];
uniform int  numLights;

// Mode control
uniform bool simpleShading = false;  // false = normal Phong (glass), true = flat (particles)
uniform bool roundParticles = false; // enable circular points when rendering GL_POINTS

out vec4 FragColor;

vec3 Ambient() {
    return ambientLight * matdiff;
}

vec3 Lambert(vec3 NormSCO, vec3 L, int i)
{
    vec3 colRes = vec3(0);
    float d = dot(L, NormSCO);
    if (d > 0)
        colRes = lightColor[i] * matdiff * d;
    return colRes;
}

vec3 Phong(vec3 NormSCO, vec3 L, vec3 vertSCO, int i)
{
    vec3 colRes = vec3(0);
    if (dot(NormSCO, L) < 0.0 || matshin == 0.0)
        return colRes;

    vec3 R = reflect(-L, NormSCO);
    vec3 V = normalize(-vertSCO);
    float shine = pow(max(dot(R, V), 0.0), matshin);
    return matspec * lightColor[i] * shine;
}

void main()
{
    // Optional roundness for point particles
    if (roundParticles) {
        vec2 uv = gl_PointCoord * 2.0 - 1.0;
        if (dot(uv, uv) > 1.0)
            discard;
    }

    vec3 color = vec3(0);

    if (simpleShading) {
        // No lighting — just use material color
        color = matdiff;
    } else {
        color = Ambient();
        for (int i = 0; i < numLights; ++i) {
            vec3 L = normalize(lightPos[i] - fvertex);
            vec3 N = normalize(fnormal);
            color += Lambert(N, L, i) + Phong(N, L, fvertex, i);
        }
    }

    FragColor = vec4(color, alpha);
}
