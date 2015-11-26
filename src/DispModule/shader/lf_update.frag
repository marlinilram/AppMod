#version 400

//! [0]
out vec4 fragColor;

uniform float fMeshSize;

void main(void)
{
		fragColor = vec4(0.5, 0.5, 0.5, 0.5);
		fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
}
//! [0]
