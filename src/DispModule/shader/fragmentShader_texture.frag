#version 400

//! [0]
uniform sampler2D text_ogl;
uniform float fMeshSize;

in vec4 vs_color;
in vec2 varyingTextureCoordinate;
in vec4 vs_rhod_irr;
in vec4 vs_rhos_specular;

out vec4 fragColor;

void main(void)
{
    //fragColor = vs_color;
	float fColor = (gl_PrimitiveID)/(fMeshSize);
	//gl_FragData[0] = vec4(fColor,0,0,0);
	//fragColor[3] = fColor;
	
	//fragColor[0] = varyingTextureCoordinate[0];
	//fragColor[1] = varyingTextureCoordinate[1];
	//fragColor[2] = 1;
	//fragColor[3] = 1;
	fragColor = texture2D(text_ogl, varyingTextureCoordinate)*vs_rhod_irr + vs_rhos_specular;
	fragColor[3] = fColor;
}
//! [0]
