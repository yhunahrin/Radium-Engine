#include "Structs.glsl"

layout (location = 0) in vec3 in_position;
layout (location = 5) in vec4 in_uc_ul;
layout (location = 6) in vec4 in_uq_p;

uniform Transform transform;

layout (location = 0) out vec3 out_position;
layout (location = 5) out vec4 out_uc_ul;
layout (location = 6) out vec4 out_uq_p;

void main()
{
    mat4 mvp = transform.proj * transform.view * transform.model;
    gl_Position = mvp * vec4(in_position, 1.0);

    out_position = in_position.xyz;
    out_uc_ul   = in_uc_ul;
    out_uq_p    = in_uq_p;
}
