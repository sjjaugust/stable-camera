#version 300 es
precision highp float;
out vec4 FragColor;
in vec2 vTexCoord;
uniform sampler2D SamplerY;
uniform sampler2D SamplerU;
void main()
{
    vec3 yuv;
    vec3 rgb;
    yuv.x = texture(SamplerY, vTexCoord).r;
    yuv.y = texture(SamplerU, vTexCoord).r - 0.5;
    yuv.z = texture(SamplerU, vTexCoord).a - 0.5;
    rgb = mat3( 1,   1,   1,
    0, -0.39465,  2.03211,
    1.13983,   -0.58060,  0) * yuv;
    FragColor = vec4(rgb, 1);
}