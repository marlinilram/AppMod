#version 400 compatibility

//! [0]
uniform mat4 gl_ModelViewProjectionMatrix;
uniform mat4 gl_ModelViewMatrix;

in vec4 vertex;
in vec4 color;
in vec3 normal;

out vec4 vs_color;
out vec3 vs_normal;
out vec4 vs_pos;
out vec3 vViewPos;

void main(void)
{
    vs_color = color;
	vs_normal = vec3(gl_ModelViewMatrix * vec4(normal,0.0));
	gl_Position = gl_ModelViewProjectionMatrix * vertex;
	vViewPos = -(gl_ModelViewMatrix * vertex).xyz;
	vs_pos = vertex;
}
//! [0]
