layout (location = 0) in vec3 in_position;
layout (location = 1) in vec3 in_normal;
layout (location = 6) in vec4 in_uc_ul;
layout (location = 5) in vec4 in_uq_p;

layout (location = 0) out vec3 out_position;
layout (location = 1) out vec3 out_normal;
layout (location = 6) out vec4 out_uc_ul;
layout (location = 5) out vec4 out_uq_p;

out vec4 color;

void main()
{
    out_position = in_position.xyz;
    out_normal = in_normal;
    out_uc_ul   = in_uc_ul;
    out_uq_p    = in_uq_p;
}
