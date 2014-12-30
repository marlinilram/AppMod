#version 400

//! [0]
in vec4 vs_color;

out vec4 fragColor;

uniform float fMeshSize;

void main(void)
{
    fragColor = vs_color;
	float fColor = (gl_PrimitiveID)/(fMeshSize);
	//gl_FragData[0] = vec4(fColor,0,0,0);
	fragColor[3] = fColor;
}
//! [0]
