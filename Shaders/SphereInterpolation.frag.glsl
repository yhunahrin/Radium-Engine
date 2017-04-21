#include "Structs.glsl"

layout (location = 0) in vec3 in_position;
layout (location = 1) in vec3 in_normal;
layout (location = 6) in vec4 in_uc_ul;
layout (location = 5) in vec4 in_uq_p;

out vec4 out_color;

uniform Material material;

void main()
{
    // color
    
    out_color = vec4(normalize(in_uc_ul.yzw)*0.5 + vec3(0.5), 1.0);
}
