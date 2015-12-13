#version 400

//! [0]
out vec4 fragColor;

uniform float fElementSize;
uniform int   renderMode;

void main(void)
{
    fragColor = vec4(0.5, 0.5, 0.5, 1.0);
    if (renderMode == 0)
    {
        //fragColor = vec4(0.5, 0.5, 0.5, 0.0);
    }
		else if (renderMode == 1)
    {
        fragColor[3] = 0;//(gl_PrimitiveID)/(fElementSize);
    }
		
}
//! [0]
