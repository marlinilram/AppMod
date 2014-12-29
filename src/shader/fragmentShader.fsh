#version 130

//! [0]
in vec4 vs_color;

out vec4 fragColor;

void main(void)
{
    fragColor = vs_color;
}
//! [0]
