#version 400 compatibility

//! [0]
uniform mat4 gl_ModelViewProjectionMatrix;

in vec4 vertex;
in vec4 color;
in vec2 textureCoordinate;
in vec4 rhod_irr;
in vec4 rhos_specular;

out vec4 vs_color;
out vec2 varyingTextureCoordinate;
out vec4 vs_rhod_irr;
out vec4 vs_rhos_specular;

void main(void)
{
    vs_color = color;
	varyingTextureCoordinate = textureCoordinate;
	vs_rhod_irr = rhod_irr;
	vs_rhos_specular = rhos_specular;
	
	gl_Position = gl_ModelViewProjectionMatrix * vertex;
}
//! [0]