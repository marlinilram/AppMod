#version 400

//! [0]
in vec4 vs_color;
in vec3 vs_normal;
in vec4 vs_pos;
in vec3 vViewPos;

out vec4 fragColor;

uniform float fMeshSize;
uniform int renderMode;
uniform vec3 L;
uniform int use_flat;

vec3 computeFaceNormal(vec3 pos)
{
	vec3 fdx = dFdx(pos);
	vec3 fdy = dFdy(pos);
	return normalize(cross(fdx, fdy));
}

void main(void)
{
	if (use_flat == 1)
	{
		vec3 n = computeFaceNormal(vViewPos);
		fragColor = vec4( .5 * ( 1. + n.x ), .5 * ( 1. + n.y ), gl_FragCoord.z, 1. );
	}
	else
	{
    switch (renderMode)
    {
    case 3:
        fragColor = vs_color;
        fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
        break;
    default:
        fragColor = vec4( .5 * ( 1. + vs_normal.x ), .5 * ( 1. + vs_normal.y ), gl_FragCoord.z, 1. );
        break;
    }
		
	}
	
	
}
//! [0]