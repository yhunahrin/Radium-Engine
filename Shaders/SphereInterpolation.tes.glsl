layout (location = 0) in vec3 in_position[];
layout (location = 6) in vec4 in_uc_ul[];
layout (location = 5) in vec4 in_uq_p[];

layout (location = 6) out vec4 out_uc_ul;
layout (location = 5) out vec4 out_uq_p;
layout (location = 0) out vec3 out_position;

layout(triangles, equal_spacing, cw) in; //ccw

void main()
{
    vec3 pos0 = gl_TessCoord.x * in_position[0];
    vec3 pos1 = gl_TessCoord.y * in_position[1];
    vec3 pos2 = gl_TessCoord.z * in_position[2];

    vec4 ucul0 = gl_TessCoord.x * in_uc_ul[0];
    vec4 ucul1 = gl_TessCoord.y * in_uc_ul[1];
    vec4 ucul2 = gl_TessCoord.z * in_uc_ul[2];

    vec4 uqp0 = gl_TessCoord.x * in_uq_p[0];
    vec4 uqp1 = gl_TessCoord.y * in_uq_p[1];
    vec4 uqp2 = gl_TessCoord.z * in_uq_p[2];

    out_uc_ul = ucul0 + ucul1 + ucul2;
    out_uq_p  = uqp0 + uqp1 + uqp2;
    out_position = pos0 + pos1 + pos2;

    gl_Position = vec4(out_position, 1.0);
    //mat4 mvp = transform.proj * transform.view * transform.model;
    //gl_Position = mvp * vec4(out_position, 1.0);
}

