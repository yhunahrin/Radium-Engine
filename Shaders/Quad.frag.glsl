in vec2 varTexcoord;
layout (location = 1) out vec4 out_normal;
layout (location = 2) out vec4 out_position;
layout (location = 3) out vec4 fragColor;

uniform bool useNormal;
uniform int neighSize;
uniform float depthThresh;
uniform vec2 WindowSize;
uniform sampler2D position;
uniform sampler2D normal;

layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_position;
float xStep, yStep;

float weight(vec2 src, vec2 neigh)
{
    //return 1;
    float squareDis = distance(src, neigh)*distance(src, neigh);
    return (squareDis-1)*(squareDis-1);
}

void main()
{
    xStep = 1/WindowSize.x;
    yStep = 1/WindowSize.y;

    float sum =0, depth = 1000;
    vec3 c = vec3(0,0,0);
  /* test
    float x = texture(useNormal ? normal : position, varTexcoord).z;
    float y = in_position.x;
    if (x == 0)
        fragColor = vec4(1, 0, 0, 1);
    else if (x < 5.999)
       fragColor = vec4(0, 1, 0, 1);
    else
        fragColor = vec4(0, 0, 1, 1);
   */
    float xStart = varTexcoord.x - neighSize * xStep, xEnd = varTexcoord.x + neighSize * xStep;
    float yStart = varTexcoord.y - neighSize * yStep, yEnd = varTexcoord.y + neighSize * yStep;
    for (float i=xStart; i <= xEnd; i+=xStep)
        for (float j=yStart; j <= yEnd; j+=yStep)
        {
            vec3 tmpColor = texture(position, vec2(i,j)).xyz;
            if (length(tmpColor) > 0.1 && tmpColor.z < depth) depth = tmpColor.z;
        }
    for (float i=xStart; i <= xEnd; i+=xStep)
        for (float j=yStart; j <= yEnd; j+=yStep)
        {
            vec3 tmpColor = texture(position, vec2(i,j)).xyz;
            vec3 tmpNormal = texture(normal, vec2(i,j)).xyz;

            if (length(tmpColor) > 0.1 && tmpColor.z - depth < depthThresh)
            {
                float w = weight(varTexcoord, vec2(i,j));
                c += w * (useNormal ? tmpNormal:tmpColor);
                sum += w;
            }
        }

    if(!useNormal) fragColor = vec4(c/sum, 1.0);
    else fragColor = vec4(c/length(c), 1.0);
    out_normal = vec4(in_normal,0.1);
    out_position = vec4(in_position, 0.1);
}
