#version 400

//! [0]
in vec2 UV;

out vec4 fragColor;

uniform sampler2D sketch_texture;
uniform float width;
uniform float height;
uniform int isBackground;
uniform float threshold;
uniform float minDepth;
uniform float maxDepth;

// X directional search matrix.
mat3 GX = mat3( -0.125, 0.0, 0.125,
                -0.25, 0.0, 0.25,
                -0.125, 0.0, 0.125 );
// Y directional search matrix.
mat3 GY =  mat3( 0.125,  0.25,  0.125,
                    0.0,  0.0,  0.0,
                -0.125, -0.25, -0.125 );

vec4 DetectEdge(in vec2 coords)
{
    vec4  fSumX = vec4( 0.0,0.0,0.0,0.0 );
    vec4  fSumY = vec4( 0.0,0.0,0.0,0.0 );
    vec4 fTotalSum = vec4( 0.0,0.0,0.0,0.0 );

    // Findout X , Y index of incoming pixel
    // from its texture coordinate.
    float fXIndex = coords.x * width;
    float fYIndex = coords.y * height;

    /* image boundaries Top, Bottom, Left, Right pixels*/
    if( ! ( fYIndex < 1.0 || fYIndex > height - 1.0 || 
            fXIndex < 1.0 || fXIndex > width - 1.0 ))
    {
        // X Directional Gradient calculation.
        for(float I=-1.0; I<=1.0; I = I + 1.0)
        {
            for(float J=-1.0; J<=1.0; J = J + 1.0)
            {
                float fTempX = ( fXIndex + I + 0.5 ) / width ;
                float fTempY = ( fYIndex + J + 0.5 ) / height ;
                vec4 fTempSumX = texture2D( sketch_texture, vec2( fTempX, fTempY ));
                fTempSumX.z = (fTempSumX.z - minDepth) / (maxDepth - minDepth);
                fSumX = fSumX + ( fTempSumX * vec4( GX[int(I+1.0)][int(J+1.0)],
                                                    GX[int(I+1.0)][int(J+1.0)],
                                                    GX[int(I+1.0)][int(J+1.0)],
                                                    GX[int(I+1.0)][int(J+1.0)]));
            }
        }

        { // Y Directional Gradient calculation.
            for(float I=-1.0; I<=1.0; I = I + 1.0)
            {
                for(float J=-1.0; J<=1.0; J = J + 1.0)
                {
                    float fTempX = ( fXIndex + I + 0.5 ) / width ;
                    float fTempY = ( fYIndex + J + 0.5 ) / height ;
                    vec4 fTempSumY = texture2D( sketch_texture, vec2( fTempX, fTempY ));
                    fTempSumY.z = (fTempSumY.z - minDepth) / (maxDepth - minDepth);
                    fSumY = fSumY + ( fTempSumY * vec4( GY[int(I+1.0)][int(J+1.0)],
                                                        GY[int(I+1.0)][int(J+1.0)],
                                                        GY[int(I+1.0)][int(J+1.0)],
                                                        GY[int(I+1.0)][int(J+1.0)]));
                }
            }
            // Combine X Directional and Y Directional Gradient.
            vec4 fTem = fSumX * fSumX + fSumY * fSumY;
            fTotalSum = sqrt( fTem );
        }
    }
    return fTotalSum;
}

vec4 MagDir(in vec2 coords)
{
    float  fSumX = 0.0;
    float  fSumY = 0.0;
    vec4 fTotalSum = vec4( 0.0,0.0,0.0,0.0 );

    // Findout X , Y index of incoming pixel
    // from its texture coordinate.
    float fXIndex = coords.x * width;
    float fYIndex = coords.y * height;

    /* image boundaries Top, Bottom, Left, Right pixels*/
    if( ! ( fYIndex < 1.0 || fYIndex > height - 1.0 || 
            fXIndex < 1.0 || fXIndex > width - 1.0 ))
    {
        // X Directional Gradient calculation.
        for(float I=-1.0; I<=1.0; I = I + 1.0)
        {
            for(float J=-1.0; J<=1.0; J = J + 1.0)
            {
                float fTempX = ( fXIndex + I + 0.5 ) / width ;
                float fTempY = ( fYIndex + J + 0.5 ) / height ;
                vec4 fTempSumX = texture2D( sketch_texture, vec2( fTempX, fTempY ));
                fTempSumX.z = (fTempSumX.z - minDepth) / (maxDepth - minDepth);
                float avg = 0.4 * fTempSumX.x + 0.4 * fTempSumX.y + 0.2 * fTempSumX.z;
                fSumX = fSumX + ( avg * GX[int(I+1.0)][int(J+1.0)] );
            }
        }

        // Y Directional Gradient calculation.
        for(float I=-1.0; I<=1.0; I = I + 1.0)
        {
            for(float J=-1.0; J<=1.0; J = J + 1.0)
            {
                float fTempX = ( fXIndex + I + 0.5 ) / width ;
                float fTempY = ( fYIndex + J + 0.5 ) / height ;
                vec4 fTempSumY = texture2D( sketch_texture, vec2( fTempX, fTempY ));
                fTempSumY.z = (fTempSumY.z - minDepth) / (maxDepth - minDepth);
                float avg = 0.4 * fTempSumY.x + 0.4 * fTempSumY.y + 0.2 * fTempSumY.z;
                fSumY = fSumY + ( avg * GY[int(I+1.0)][int(J+1.0)] );
            }
        }
        // Combine X Directional and Y Directional Gradient.
        float mag = sqrt(fSumX * fSumX + fSumY * fSumY);
        float dir = atan(fSumY, fSumX);
        fTotalSum = vec4(fSumX, fSumY, mag, dir);
    }
    return fTotalSum;
}

vec4 CannySearch(in vec2 coords)
{
    vec4 magdir = texture2D( sketch_texture, coords );
    float alpha = 0.5 / sin(3.14159 / 8);

    vec2 offset = vec2(round( alpha * magdir.x / magdir.z ), round( alpha * magdir.y / magdir.z ));

    vec4 fwdneighbour, backneighbour;
    fwdneighbour = texture2D( sketch_texture, coords + offset );
    backneighbour = texture2D( sketch_texture, coords - offset );

    vec4 color;
    if (fwdneighbour.z > magdir.z || backneighbour.z > magdir.z)
    {
        color = vec4(0.0, 0.0, 0.0, 0.5);
    }
    else
    {
        color = vec4(1.0, 1.0, 1.0, 1.0);
    }
    if (magdir.z < threshold)
    {
        color = vec4(0.0, 0.0, 0.0, 0.5);
    }
    return color;
}

void main(void)
{
    if (isBackground == 1)
    {
        fragColor = texture2D(sketch_texture, UV);
    }
    else if (isBackground == 0)
    {
        //vec4 color = vec4(0.0,0.0,0.0,0.5);
        //vec4 response = DetectEdge(UV);
        //if (response.x > threshold || response.y > threshold || response.z > threshold)
        //{
        //	color.g = 1.0;
        //	color.w = 1.0;
        //}
        //fragColor = response;
        //fragColor.w = 1.0;
        //fragColor = texture2D(sketch_texture, UV);
        //fragColor.z = (fragColor.z - minDepth) / (maxDepth - minDepth);
        //fragColor.w = 1.0;
        //fragColor.x = fragColor.z;
        //fragColor.y = fragColor.z;
        //fragColor = color;
        fragColor = MagDir(UV);
    }
    else if (isBackground == 2)
    {
        fragColor = CannySearch(UV);
    }
}
//! [0]