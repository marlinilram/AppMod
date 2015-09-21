#version 400 compatibility

//! [0]
uniform mat4 gl_ModelViewProjectionMatrix;

in vec4 vertex;
in vec4 color;
in vec3 normal;

out vec4 vs_color;
out vec3 vs_normal;
out vec4 vs_pos;

void main(void)
{
    vs_color = color;
	vs_normal = normal;
	gl_Position = gl_ModelViewProjectionMatrix * vertex;
	vs_pos = vertex;
}
//! [0]
