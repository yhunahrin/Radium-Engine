#ifndef PROGRESSIVEMESH_H
#define PROGRESSIVEMESH_H


#include <Core/Mesh/MeshTypes.hpp>
#include <Core/Mesh/MeshUtils.hpp>
#include <Core/Mesh/TriangleMesh.hpp>
#include <Core/Mesh/DCEL/Dcel.hpp>

#include <Core/Mesh/ProgressiveMesh/PriorityQueue.hpp>
#include <Core/Mesh/ProgressiveMesh/ProgressiveMeshData.hpp>
#include <Core/Mesh/ProgressiveMesh/ErrorMetric.hpp>

#include <Core/Math/Quadric.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Math/Quadric.hpp>

#include <Core/Index/Index.hpp>

#include <Core/Containers/VectorArray.hpp>

#include <iostream>
#include <fstream>

namespace Ra
{
    namespace Core
    {

        //template<class ErrorMetric = QuadricErrorMetric>
        template<class ErrorMetric = SimpleAPSSErrorMetric>
        class ProgressiveMeshBase
        {
        public:

            using Primitive = typename ErrorMetric::Primitive;

            virtual void constructPriorityQueue(PriorityQueue& queue,  std::ofstream &file) = 0;

            virtual void computeVerticesPrimitives() = 0;

            virtual std::vector<ProgressiveMeshData> constructM0(int targetNbFaces, int &nbNoFrVSplit, int primitiveUpdate, Scalar scale, Scalar gradient_weight, int weight_per_vertex) = 0;

            virtual void vsplit(ProgressiveMeshData pmData) = 0;
            virtual void ecol(ProgressiveMeshData pmData) = 0;

            virtual Dcel* getDcel() = 0;
            virtual int getNbFaces() = 0;
            virtual ErrorMetric getEM() = 0;
            virtual Primitive getPrimitive(int i) = 0;
        };

        //template<class ErrorMetric = QuadricErrorMetric>
        template<class ErrorMetric = SimpleAPSSErrorMetric>
        class ProgressiveMesh : public ProgressiveMeshBase<ErrorMetric>
        {

        public:

            using Primitive = typename ErrorMetric::Primitive;

            ProgressiveMesh(TriangleMesh* mesh);
            ProgressiveMesh(const ProgressiveMesh& mesh) {}

            ~ProgressiveMesh() {}

            /// We construct a priority queue with an error for each edge
            void constructPriorityQueue(PriorityQueue& queue, std::ofstream &file);

            ///
            void projectOnAlgebraicSphereSurface();

            ///
            void cleaning(HalfEdge_ptr he);

            /// Construction of the coarser mesh
            std::vector<ProgressiveMeshData> constructM0(int targetNbFaces, int &nbNoFrVSplit, int primitiveUpdate, Scalar scale, Scalar gradient_weight, int weight_per_vertex) override;

            /// Vertex Split and Edge Collapse
            void vsplit(ProgressiveMeshData pmData) override;
            void ecol(ProgressiveMeshData pmData) override;

            /// ComputeVertexQuadric
            inline void computeVerticesPrimitives();

            /// Compute the size of the bounding box of a mesh
            void computeBoundingBoxSize(Scalar &min_x, Scalar &max_x, Scalar &min_y, Scalar &max_y, Scalar &min_z, Scalar &max_z);

            /// Compute mean edge length
            void computeMeanEdgeLength();

            /// Getters and Setters
            inline Dcel* getDcel();
            inline int getNbFaces();
            inline ErrorMetric getEM();
            inline Primitive getPrimitive(int i);

        private:

            /// Compute wedge angle
            Scalar getWedgeAngle(Index faceIndex, Index vIndex);

            /// Combine primitives
            Primitive combine(const std::vector<Primitive>& primitives, const std::vector<Scalar>& weightsWedgeAngles);

            /// Check if an edge collapse is doable
            bool checkConsistency(FaceList adjFaces, Vertex_ptr v, Face_ptr f1, Face_ptr f2, Vector3 pResult, bool &consistent);
            bool isEcolConsistent(Index halfEdgeIndex, const Vector3 &pResult);
            bool isEcolPossible(Index halfEdgeIndex, const Vector3& pResult, Primitive& q);

            /// Updates
            void updateVerticesPrimitives(Index vsIndex, HalfEdge_ptr he);
            void updatePriorityQueue(PriorityQueue &pQueue, Index vsId, Index vtId, std::ofstream &file);


        private:
            Dcel* m_dcel;
            std::vector<Primitive> m_primitives_he; //TO REMOVE
            std::vector<Primitive> m_primitives_v;
            ErrorMetric m_em;

            Scalar m_bbox_size;
            Scalar m_mean_edge_length;
            Scalar m_scale;
            Scalar m_gradient_weight;
            unsigned int m_ring_size;
            int m_weight_per_vertex;
            int m_primitive_update;

            int m_nb_faces;
            int m_nb_vertices;
        };
    }

}

#include <Core/Mesh/ProgressiveMesh/ProgressiveMesh.inl>

#endif // PROGRESSIVEMESH_H
