#version 400

//! [0]
in vec4 vs_color;
in vec3 vs_normal;
in vec4 vs_pos;

out vec4 fragColor;

uniform float fMeshSize;
uniform int renderMode;
uniform vec3 L;

void main(void)
{
	switch (renderMode)
	{
	case 3:
		fragColor = vs_color;
		fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
		break;
	case 0:
	    float shading = max(dot(normalize(vs_normal),L), 0.0);  
		vec4 Idiff = vec4(shading, shading, shading, 0.0);
		Idiff = clamp(Idiff, 0.0, 1.0);
		fragColor = Idiff;
		fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
		break;
	case 1:
		vec4 normColor = vec4(vs_normal, 0);
		fragColor = (normColor + 1) / 2;
		fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
		break;
	case 2:
		fragColor = (vec4(vs_normal, 0) + 1) / 2;
		fragColor[3] = 0.75;
		break;
	default:
		break;
	}
	
}
//! [0]
