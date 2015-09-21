#version 400
//! [0]

in vec4 vertex;

out vec2 UV;

void main(void)
{
	gl_Position = vertex;
	UV = (vertex.xy + vec2(1.0, 1.0)) / 2.0;
}
//! [0]
