#include "Structs.glsl"

layout (location = 0) in vec3 in_position;
layout (location = 5) in vec4 in_uc_ul;
layout (location = 6) in vec4 in_uq_p;
in vec3 barycentric;
out vec4 out_color;

//-----------------------------------------------------

float prattNorm(float uc, float uq)
{
    return sqrt(1.0 - 4.0*uc*uq);
}
//line 90
float potential(vec3 q, float uc, vec3 ul, float uq, vec3 p)
{  
    vec3 lq = q - p;
    return uc + dot(lq, ul) + uq * lq.length() * lq.length();
}

vec3 project(vec3 q, float uc, vec3 ul, float uq, vec3 p)
{
    // turn to centered basis
    vec3 lq = q - p; //line 100

    vec3 grad = vec3(0.0, 0.0, 0.0);
    vec3 dir        = ul + 2.0 * uq * lq;
    float ilg      = 1.0 / dir.length();
    dir             = ilg * dir;
    float ad       = uc + dot(ul, lq) + uq * lq.length() * lq.length();
    float delta    = -ad * min(ilg, 1.0);
    vec3 proj       = lq + dir * delta;

    for (int i = 0; i < 16; ++i)
    {
        grad  = ul + 2.0 * uq * proj;
        ilg   = 1.0 / grad.length();
        delta = -(uc + dot(proj, ul) + uq * proj.length() * proj.length()) * min(ilg, 1.);
        proj += dir * delta;
    }
    return proj + p;
}

//-----------------------------------------------------
float distanceFaceSphere(float uc, vec3 ul, float uq, vec3 p, vec3 v0, vec3 v1, vec3 v2)
{
    vec3 v0_centered = v0 - p;
    vec3 v1_centered = v1 - p;
    vec3 v2_centered = v2 - p;

    float prim_uc = 0.0;
    vec3 prim_ul = (1.0/6.0) * ul + (1.0/3.0) * uq * v2_centered;
    float prim_uq = (1.0/12.0) * uq;
    vec3 prim_p = vec3(0.0, 0.0, 0.0);

    float residual = (1.0/12.0) * uq * dot((v1_centered - v2_centered), (v0_centered - v2_centered));
    float area = ( cross(( v0 - v1 ), (v2 - v0)) ).length() * 0.5;

    float dist = 2.0 * area * (0.5 * potential(v2, uc, ul, uq, p) + potential(v0 - v2, prim_uc, prim_ul, prim_uq, prim_p) + potential(v1 - v2, prim_uc, prim_ul, prim_uq, prim_p) + residual) ;
    return dist;
}

//-----------------------------------------------------
void main()
{
    // apply norm
    vec3 new_ul = in_uc_ul.yzw / in_uc_ul.yzw.length();
    float pratt = prattNorm(in_uc_ul.x, in_uq_p.x);
    float new_uc = in_uc_ul.x / pratt;
    new_ul = new_ul / pratt;
    float new_uq = in_uq_p.x / pratt;
    vec3 new_p = in_uq_p.yzw;
 
    // error = distance between adj-faces and the sphere
    vec3 vgamma = project(in_position, new_uc, new_ul, new_uq, new_p);
    float color = abs(potential(vgamma, new_uc, new_ul, new_uq, new_p)) / 2.0; // TODO having the adjacent faces

    // color
    out_color = vec4(1.0 - color, color, 0.0, 1.0);
}
