layout (vertices = 3) out;

layout (location = 0) in vec3 in_position[];
layout (location = 1) in vec3 in_normal[];
layout (location = 6) in vec4 in_uc_ul[];
layout (location = 5) in vec4 in_uq_p[];

layout (location = 0) out vec3 out_position[];
layout (location = 1) out vec3 out_normal[];
layout (location = 6) out vec4 out_uc_ul[];
layout (location = 5) out vec4 out_uq_p[];

void main(void)
{

    out_position[gl_InvocationID] = in_position[gl_InvocationID];
    out_normal[gl_InvocationID] = in_normal[gl_InvocationID];
	out_uc_ul[gl_InvocationID] = in_uc_ul[gl_InvocationID];
    out_uq_p[gl_InvocationID] = in_uq_p[gl_InvocationID];

    float TessLevelInner = 2;
    float TessLevelOuter = 2;

    if (gl_InvocationID == 0)
    {
        gl_TessLevelInner[0] = TessLevelInner;
        gl_TessLevelOuter[0] = TessLevelOuter;
        gl_TessLevelOuter[1] = TessLevelOuter;
        gl_TessLevelOuter[2] = TessLevelOuter;
    }
}
