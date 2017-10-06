#include "Structs.glsl"

uniform Material material;
uniform Light light;
uniform sampler2D uShadowMap;

out vec4 fragColor;

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec3 in_normal;
layout (location = 2) in vec3 in_texcoord;
layout (location = 3) in vec3 in_eye;
layout (location = 4) in vec3 in_tangent;

#include "LightingFunctions.glsl"

void main()
{
    if (toDiscard()) discard;

    //vec3 color = getKd() * 0.1 + (1 - Shadow()) * computeLighting();
    vec3 color = computeLighting();

#if 1
    const float threshold = 0.75;
    float u = in_texcoord.x;
    float udx = dFdx( in_texcoord.x );
    float udy = dFdy( in_texcoord.x );

#if 0
    float[9] values = float[]( u-udx-udy, u-udy, u+udx-udy,
                               u-udx    , u    , u+udx    ,
                               u-udx+udy, u+udy, u+udx+udy );
    float s = 0.0;
    for (int i=0; i<9; ++i) {
        s += sign( values[i] - threshold );
    }
    float boost = 1;
    float alpha = clamp( boost*(1.0-abs(s)/9.0), 0.0, 1.0 );
#else
    const int width = 7;
    float[width*width] values;
    int k = 0;
    for (int i=-width/2; i<width/2+1; ++i)
    {
        for (int j=-width/2; j<width/2+1; ++j)
        {
            values[k++] = u + j*udx + i*udy;
        }
    }
    float s = 0.0;
    for (int i=0; i<width*width; ++i) {
        s += sign( values[i] - threshold );
    }
    float boost = 0.5;
    float alpha = clamp( boost*(1.0-abs(s)/float(width*width)), 0.0, 1.0 );
#endif

    if (u > threshold)
    {
        fragColor = alpha * vec4(color, 1.0);
    }
    else
    {
        fragColor = vec4(color, 1.0);
    }

#else
    if (in_texcoord.x > 0.9)
    {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
    }
    else
    {
        fragColor = vec4(color, 1.0);
    }
#endif

}
