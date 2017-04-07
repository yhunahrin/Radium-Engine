layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

layout (location = 0) in vec3 in_position[];
layout (location = 5) in vec4 in_uc_ul[];
layout (location = 6) in vec4 in_uq_p[];

layout (location = 5) out vec4 out_uc_ul;
layout (location = 6) out vec4 out_uq_p;
layout (location = 0) out vec3 out_position;

void main(void)
{

    gl_Position = gl_in[0].gl_Position;
    out_position = in_position[0];
	out_uc_ul = in_uc_ul[0];
    out_uq_p = in_uq_p[0];
	EmitVertex();


	gl_Position = gl_in[1].gl_Position;
    out_position = in_position[1];
	out_uc_ul = in_uc_ul[1];
    out_uq_p = in_uq_p[1];
	EmitVertex();


	gl_Position = gl_in[2].gl_Position;
    out_position = in_position[2];
	out_uc_ul = in_uc_ul[2];
    out_uq_p = in_uq_p[2];
	EmitVertex();

	EndPrimitive();
}
