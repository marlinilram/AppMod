#version 400 compatibility

//! [0]
uniform mat4 gl_ModelViewProjectionMatrix;
uniform mat4 gl_ModelViewMatrix;
uniform mat4 rigidTransform;

in vec4 vertex;

void main(void)
{
  // need to multiply the ICP Rigid Transform here
	gl_Position = gl_ModelViewProjectionMatrix * rigidTransform * vertex;
}
//! [0]