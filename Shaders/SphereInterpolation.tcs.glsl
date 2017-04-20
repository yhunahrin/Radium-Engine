layout (vertices = 3) out;

layout (location = 0) in vec3 in_position[];
layout (location = 6) in vec4 in_uc_ul[];
layout (location = 5) in vec4 in_uq_p[];

layout (location = 6) out vec4 out_uc_ul[];
layout (location = 5) out vec4 out_uq_p[];
layout (location = 0) out vec3 out_position[];

#define ID gl_InvocationID

void main(void)
{

    out_position[ID] = in_position[ID];
	out_uc_ul[ID] = in_uc_ul[ID];
    out_uq_p[ID] = in_uq_p[ID];

    float TessLevelInner = 2;
    float TessLevelOuter = 2;

    if (ID == 0)
    {
        gl_TessLevelInner[0] = TessLevelInner;
        gl_TessLevelOuter[0] = TessLevelOuter;
        gl_TessLevelOuter[1] = TessLevelOuter;
        gl_TessLevelOuter[2] = TessLevelOuter;
    }
}
