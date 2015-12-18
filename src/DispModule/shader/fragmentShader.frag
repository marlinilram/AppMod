#version 400

//! [0]
in vec4 vs_color;
in vec3 vs_normal;
in vec3 vs_eye_normal;
in vec4 vs_pos;
in float vs_CrestTag;
in vec2 vs_uv;
in float vs_SynTextureTag;

out vec4 fragColor;

uniform float fMeshSize;
uniform int renderMode;
uniform vec3 L;
uniform float threshold;
uniform sampler2D reflect_texture;
uniform sampler2D syn_reflect_texture;

float edgeFactor(){
    float d = fwidth(vs_CrestTag);
    float a = smoothstep(0.0, d*1.5, vs_CrestTag);
    return a;
}

void main(void)
{
	switch (renderMode)
	{
  case 5:
		fragColor = (vec4(vs_eye_normal, 0) + 1) / 2;
		fragColor[3] = (gl_PrimitiveID)/(fMeshSize);
    break;
  case 4:
    fragColor = vs_color;
    //fragColor = vec4(vs_CrestTag);
    //fragColor[3] = 1.0;
    //if (edgeFactor() < 5 * threshold)
    //{
    //  fragColor = (vec4(vs_normal, 0) + 1) / 2;
    //  fragColor[3] = 0.75;
    //}
    //else
    //{
    //  fragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
    //}
    break;
	case 3:
    if (vs_SynTextureTag <= 0)
    {
      vec4 texton = texture2D( reflect_texture, vs_uv);
		  fragColor = texton;
    }
    else if (vs_SynTextureTag > 0)
    {
      vec4 texton = texture2D( syn_reflect_texture, vs_uv);
		  fragColor = texton;
    }
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
		fragColor = vec4(0.0, 0.0, 0.0, 0.3);
		//fragColor[3] = 0.75;
		break;
	default:
		break;
	}
	
}
//! [0]
