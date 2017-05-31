#include "Structs.glsl"

uniform Transform transform;

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

layout (location = 0) in vec3 in_position[];
layout (location = 1) in vec3 in_normal[];
layout (location = 6) in vec4 in_uc_ul[];
layout (location = 5) in vec4 in_uq_p[];

layout (location = 0) out vec3 out_position;
layout (location = 1) out vec3 out_normal;
layout (location = 6) out vec4 out_uc_ul;


float prattNorm(float uc, vec3 ul, float uq)
{
    float ul_length = length(ul);
    return sqrt(ul_length*ul_length - 4.0*uc*uq);
}

vec3 project(vec3 q, float uc, vec3 ul, float uq, vec3 p)
{
    // turn to centered basis
    vec3 lq = q - p;

    vec3 grad       = vec3(0.0, 0.0, 0.0);
    vec3 dir        = ul + 2.0 * uq * lq;
    float ilg       = 1.0 / length(dir);
    dir             = ilg * dir;
    float ad        = uc + dot(ul, lq) + uq * length(lq) * length(lq);
    float delta     = -ad * min(ilg, 1.0);
    vec3 proj       = lq + dir * delta;

    for (int i = 0; i < 16; ++i)
    {
        grad    = ul + 2.0 * uq * proj;
        ilg     = 1.0 / length(grad);
        delta   = -(uc + dot(proj, ul) + uq * length(proj) * length(proj)) * min(ilg, 1.);
        proj    += dir * delta;
    }

    return proj + p;
}

void main(void)
{
    for (int i = 0; i < 3; i++)
    {
        float pratt = prattNorm(in_uc_ul[i].x, in_uc_ul[i].yzw, in_uq_p[i].x);
        float uc = in_uc_ul[i].x / pratt;
        vec3 ul = vec3(in_uc_ul[i].y, in_uc_ul[i].z, in_uc_ul[i].w) / pratt;
        float uq = in_uq_p[i].x / pratt;
        vec3 p = in_uq_p[i].yzw;

        //vec3 vgamma = project(in_position[i], new_uc, new_ul, new_uq, new_p);
        //vec3 vgamma = in_uc_ul[i].yzw;
        vec3 vgamma = p; //project(in_position[i], uc, ul, uq, p);

        out_position = vgamma;
        out_normal = in_normal[i];
        out_uc_ul = vec4(uc, ul);

        mat4 mvp = transform.proj * transform.view * transform.model;
        gl_Position = mvp * vec4(out_position, 1.0);

	    EmitVertex();
    }

	EndPrimitive();
}
