#ifndef ERROR_METRIC_H
#define ERROR_METRIC_H

#include <Core/Math/Quadric.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Mesh/DCEL/Dcel.hpp>
#include <Core/Mesh/ProgressiveMesh/AlgebraicSphere.hpp>
#include <Patate/grenaille.h>

#include <iostream>
#include <fstream>

namespace Ra
{
    namespace Core
    {
        class QuadricErrorMetric
        {

        public:
            using Primitive = Quadric<3>;

            QuadricErrorMetric();
            QuadricErrorMetric(Scalar scale);

            Scalar computeError         (HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q);
            Scalar computeGeometricError(const Primitive& q, const Vector3& p);

            Primitive combine(const std::vector<Primitive>& p, const std::vector<Scalar>& weights, Scalar normalizing_weight_factor);

            Scalar getWedgeAngle(Face_ptr face, Vertex_ptr v);

            void generateVertexPrimitive    (Primitive &q, Vertex_ptr v, Scalar weight, int ringSize);
            void generateFacePrimitive      (Primitive &q, Face_ptr f, Scalar weight, int ringSize);
        };

        //---------------------------------------------------

        using namespace Grenaille;
        class GrenaillePoint
        {
        public:

            enum {Dim = 3};
            typedef typename ::Scalar Scalar;
            typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;

            inline GrenaillePoint(const VectorType &pos = VectorType::Zero(),
                                  const VectorType &normal = VectorType::Zero())
                : m_pos(pos), m_normal(normal)
            {}

            inline const VectorType& pos() const
            {
                return m_pos;
            }

            inline const VectorType& normal() const
            {
                return m_normal;
            }

            inline void setPos(VectorType &pos)
            {
                m_pos = pos;
            }

            inline void setNormal(VectorType &normal)
            {
                m_normal = normal;
            }


            inline void setPos(const Vector3 &v)
            {
                m_pos[0] = v.x();
                m_pos[1] = v.y();
                m_pos[2] = v.z();
            }

            inline void setNormal(const Vector3 &n)
            {
                m_normal[0] = n.x();
                m_normal[1] = n.y();
                m_normal[2] = n.z();
            }


        private:
            VectorType m_pos, m_normal;
        };


        /// On tente de venir projeter les point résultant sur
        /// la sphère moyenne
        class SimpleAPSSErrorMetric
        {
            //typedef DistWeightFunc<GrenaillePoint, ConstantWeightKernel<Scalar> > WeightFunc;
            typedef DistWeightFunc<GrenaillePoint, SmoothWeightKernel<Scalar> > WeightFunc;
            typedef Basket<GrenaillePoint, WeightFunc, OrientedSphereFit, GLSParam, APSSInterpolation> Fit1;
            //typedef Basket<GrenaillePoint, WeightFunc, UnorientedSphereFit, GLSParam> Fit2;
            //typedef Basket<GrenaillePoint, WeightFunc, OrientedSphereFit, GLSParam, OrientedSphereSpaceDer, GLSDer, GLSCurvatureHelper> Fit3;

        public:
            using Primitive = Fit1;

            SimpleAPSSErrorMetric();

            Scalar computeError         (HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q, Scalar gradient_weight, Scalar min_radius, Scalar max_radius, std::ofstream &file);
            Scalar computeEdgeMinErrorOnEdge  (HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q, Scalar gradient_weight, Scalar min_radius, Scalar max_radius, std::ofstream &file);
            Scalar computeEdgeMinErrorOnFace  (HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q, Scalar gradient_weight, Scalar min_radius, Scalar max_radius, std::ofstream &file);

            Scalar computeAlpha(HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q);
            Scalar computeUV(HalfEdge_ptr he, std::vector<Primitive>& v_primitives, Vector3& pResult, Primitive &q);

            Scalar computeGeometricError(const Primitive& q, const Vector3& p);

            void generateRIMLSVertexPrimitive(Primitive &q, Vertex_ptr v, int ringSize);
            void generateVertexPrimitive    (Primitive &q, Vertex_ptr v, Scalar weight, int ringSize);

            void generateFacePrimitive      (Primitive &q, Face_ptr f, Scalar weight, int ringSize);
        };

    }




}

#include <Core/Mesh/ProgressiveMesh/ErrorMetric.inl>

#endif // ERROR_METRIC_H
