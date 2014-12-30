#version 400 compatibility

//! [0]
uniform mat4 gl_ModelViewProjectionMatrix;

in vec4 vertex;
in vec4 color;

out vec4 vs_color;

void main(void)
{
    vs_color = color;
	gl_Position = gl_ModelViewProjectionMatrix * vertex;
}
//! [0]
