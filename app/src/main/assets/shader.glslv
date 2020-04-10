#version 300 es
precision highp float;
in vec3 vPosition;
in vec2 inTexCoord;
out vec2 vTexCoord;
uniform mat3 transform;
void main() {
    mat3 root_trans = mat3(0.0, -2.0/1920.0, 0.0, -2.0/1080.0, 0.0, 0.0,  1.0, 1.0, 1.0);
    mat3 root_trans_i = mat3( 0.0, -1080.0/2.0, 0.0, -1920.0 / 2.0, 0.0, 0.0, 1920.0/2.0, 1080.0/2.0, 1.0);
    mat3 scale_trans = mat3(1.1, 0.0, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 1.0);

    vec3 pos_3 = scale_trans * root_trans * transform * root_trans_i * vec3(vPosition.xy, 1.0);
    gl_Position = vec4(pos_3.xy, 0.0, 1.0);
    vTexCoord = inTexCoord;
}