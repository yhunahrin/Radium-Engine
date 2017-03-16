#include "ProgressiveMesh.hpp"

#include <Core/RaCore.hpp>

#include <Core/Log/Log.hpp>

#include <Core/Mesh/Wrapper/Convert.hpp>

#include <Core/Mesh/DCEL/Vertex.hpp>
#include <Core/Mesh/DCEL/HalfEdge.hpp>
#include <Core/Mesh/DCEL/FullEdge.hpp>
#include <Core/Mesh/DCEL/Operations/EdgeOperation.hpp>
#include <Core/Mesh/DCEL/Operations/VertexOperation.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VVIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VFIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VHEIterator.hpp>
#include <Core/Mesh/DCEL/Iterator/Edge/EFIterator.hpp>

#include <Core/Mesh/ProgressiveMesh/PriorityQueue.hpp>

#include <Core/Geometry/Triangle/TriangleOperation.hpp>

namespace Ra
{
    namespace Core
    {

        template<class ErrorMetric>
        ProgressiveMesh<ErrorMetric>::ProgressiveMesh(TriangleMesh* mesh)
        {
            m_dcel = new Dcel();
            m_em = ErrorMetric();
            m_nb_faces = mesh->m_triangles.size();
            m_nb_vertices = mesh->m_vertices.size();

            convert(*mesh, *m_dcel);

            m_mean_edge_length = Ra::Core::MeshUtils::getMeanEdgeLength(*mesh);
            m_scale = 0.0;
            m_ring_size = 0;
            m_weight_per_vertex = 0;
            m_primitive_update = 0;
        }

        //------------------------------------------------

        template <class ErrorMetric>
        inline Dcel* ProgressiveMesh<ErrorMetric>::getDcel()
        {
            return m_dcel;
        }

        template <class ErrorMetric>
        inline int ProgressiveMesh<ErrorMetric>::getNbFaces()
        {
            return m_nb_faces;
        }

        template <class ErrorMetric>
        inline ErrorMetric ProgressiveMesh<ErrorMetric>::getEM()
        {
            return m_em;
        }

        //------------------------------------------------

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::computeBoundingBoxSize(Scalar &min_x, Scalar &max_x, Scalar &min_y, Scalar &max_y, Scalar &min_z, Scalar &max_z)
        {
            min_x = max_x = m_dcel->m_vertex[0]->P().x();
            min_y = max_y = m_dcel->m_vertex[0]->P().y();
            min_z = max_z = m_dcel->m_vertex[0]->P().z();
            for (unsigned int i = 0; i < m_dcel->m_vertex.size(); i++)
            {
                if (m_dcel->m_vertex[i]->P().x() < min_x) min_x = m_dcel->m_vertex[i]->P().x();
                if (m_dcel->m_vertex[i]->P().x() > max_x) max_x = m_dcel->m_vertex[i]->P().x();
                if (m_dcel->m_vertex[i]->P().y() < min_y) min_y = m_dcel->m_vertex[i]->P().y();
                if (m_dcel->m_vertex[i]->P().y() > max_y) max_y = m_dcel->m_vertex[i]->P().y();
                if (m_dcel->m_vertex[i]->P().z() < min_z) min_z = m_dcel->m_vertex[i]->P().z();
                if (m_dcel->m_vertex[i]->P().z() > max_z) max_z = m_dcel->m_vertex[i]->P().z();
            }
            Vector3 size = Vector3(max_x-min_x, max_y-min_y, max_z-min_z);
            //Vector3 center = Vector3((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);
            m_bbox_size = size.norm();
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::computeMeanEdgeLength()
        {
            Scalar mean_edge_length = 0.0;
            unsigned int nb_edges = 0;
            // TODO parallelization
            for (unsigned int i = 0; i < m_dcel->m_face.size(); i++)
            {
                Face_ptr f = m_dcel->m_face[i];
                if (f->HE() == NULL) continue;
                HalfEdge_ptr he = f->HE();
                mean_edge_length += (he->V()->P() - he->Next()->V()->P()).norm();
                nb_edges++;
                he = he->Next();
                mean_edge_length += (he->V()->P() - he->Next()->V()->P()).norm();
                nb_edges++;
                he = he->Next();
                mean_edge_length += (he->V()->P() - he->Next()->V()->P()).norm();
                nb_edges++;
            }
            mean_edge_length /= nb_edges;
            m_mean_edge_length = mean_edge_length;
        }

        //-----------------------------------------------------

        template<class ErrorMetric>
        typename ErrorMetric::Primitive ProgressiveMesh<ErrorMetric>::combine(const std::vector<Primitive>& primitives, const std::vector<Scalar>& weightsWedgeAngles)
        {
            std::vector<Scalar> weights;
            weights.reserve(primitives.size());
            Scalar normalizing_weight_factor = 1.0;

            if (m_weight_per_vertex == 0) // equal
            {
                for (unsigned int i = 0; i < primitives.size(); i++)
                {
                    weights.push_back(1.0);
                }
                normalizing_weight_factor = primitives.size();
            }
            else if (m_weight_per_vertex == 1) // wedge angle
            {
                Scalar sumWedgeAngles = 0.0;
                for (unsigned int i = 0; i < primitives.size(); i++)
                {
                    sumWedgeAngles += weightsWedgeAngles[i];
                }
                normalizing_weight_factor = sumWedgeAngles;
                for (unsigned int i = 0; i < primitives.size(); i++)
                {
                    weights.push_back(weightsWedgeAngles[i]);
                }
            }
            return m_em.combine(primitives, weights, normalizing_weight_factor);
        }

        //-----------------------------------------------------

        template<class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::computeVerticesPrimitives()
        {
            const uint numVertices = m_dcel->m_vertex.size();
            const uint numFaces = m_dcel->m_face.size();

            m_primitives_he.clear();
            m_primitives_he.reserve(numFaces*3);
            m_primitives_v.clear();
            m_primitives_v.reserve(numVertices);

            Scalar progression = 0.0;
//#pragma omp parallel for
            Primitive q;
            for (uint v = 0; v < numVertices; ++v)
            {
                if (std::abs(Scalar(Scalar(v) / Scalar(numVertices)) - progression * 0.10) < (1.0/numVertices))
                {
                    LOG(logINFO) << progression * 10 << "% done";
                    progression += 1.0;
                }

                m_em.generateVertexPrimitive(q, m_dcel->m_vertex[v], m_scale, m_ring_size);
                //m_em.generateRIMLSVertexPrimitive(q, m_dcel->m_vertex[v], m_ring_size);
//#pragma omp critical
                m_primitives_v.push_back(q);
            }
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::updateVerticesPrimitives(Index vsIndex, HalfEdge_ptr he)
        {
            // We go all over the faces which contain vsIndex
            VVIterator vvIt = VVIterator(m_dcel->m_vertex[vsIndex]);
            VertexList adjVertices = vvIt.list();

            if (m_primitive_update == 0) // re-calcul
            {
                for (uint t = 0; t < adjVertices.size(); ++t)
                {
                    Primitive q;
                    m_em.generateVertexPrimitive(q, adjVertices[t], m_scale, m_ring_size);
                    //m_em.generateRIMLSVertexPrimitive(q, adjVertices[t], m_ring_size);
                    m_primitives_v[adjVertices[t]->idx] = q;
                }
            }
            else if (m_primitive_update == 1) // no update
            {
                m_primitives_v[vsIndex] = m_primitives_he[he->idx];
            }
        }

        //-----------------------------------------------------

        template <class ErrorMetric>
        Scalar ProgressiveMesh<ErrorMetric>::getWedgeAngle(Index faceIndex, Index vIndex)
        {
            Scalar wedgeAngle;
            Face_ptr face = m_dcel->m_face[faceIndex];
            Vertex_ptr v = m_dcel->m_vertex[vIndex];

            HalfEdge_ptr he = face->HE();
            for (int i = 0; i < 3; i++)
            {
                if (he->V() == v)
                {
                    Vector3 v0 = he->Next()->V()->P() - he->V()->P();
                    Vector3 v1 = he->Prev()->V()->P() - he->V()->P();
                    v0.normalize();
                    v1.normalize();
                    wedgeAngle = std::acos(v0.dot(v1));
                    break;
                }
                he = he->Next();
            }
            return wedgeAngle;
        }

        //--------------------------------------------------

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::constructPriorityQueue(PriorityQueue &pQueue)
        {
            const uint numTriangles = m_dcel->m_face.size();
            pQueue.reserve(numTriangles*3 / 2);
            //std::ofstream file ("error_pqueue.dat", std::ofstream::out);

           // Scalar progression = 0.0;
            //uint nb = 0;
            Primitive prims [3];
            PriorityQueue::PriorityQueueData data[3];
            Index ids[3];
            Scalar progression = 0.0;
            uint i_multi_thread = 0;
#pragma omp parallel for private(prims, data, ids)
            for (unsigned int i = 0; i < numTriangles; i++)
            {
                const Face_ptr& f = m_dcel->m_face.at( i );
                HalfEdge_ptr h = f->HE();
                for (int j = 0; j < 3; j++)
                {
                    const Vertex_ptr& vs = h->V();
                    const Vertex_ptr& vt = h->Next()->V();

                    // To prevent adding twice the same edge
                    if (vs->idx > vt->idx)
                    {
                        h = h->Next();
                        ids[j] = Index::INVALID_IDX();
                        continue;
                    }

                    Vector3 p = Vector3::Zero();
                    double edgeError = m_em.computeError(h, m_primitives_v, p, prims[j]);
                    data[j] = PriorityQueue::PriorityQueueData(vs->idx, vt->idx, h->idx, i, edgeError, p);
                    ids[j] = h->idx;

                    h = h->Next();
                }
#pragma omp critical
                {
                    for (int j = 0; j < 3; j++){
                        if (ids[j].isValid() ) {
                            m_primitives_he[ids[j]] = prims[j];
                            pQueue.insert(data[j]);
                        }
                    }

                    if (std::abs(Scalar(Scalar(i_multi_thread++) / Scalar(numTriangles)) - progression * 0.10) < (1.0/numTriangles))
                    {
                        LOG(logINFO) << progression * 10 << "% done";
                        progression += 1;
                    }
                }
            }
            projectOnAlgebraicSphereSurface();
            //file.close();
            //pQueue.display();
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::projectOnAlgebraicSphereSurface()
        {
            for (unsigned int i = 0; i < m_dcel->m_vertex.size(); i++)
            {
                Vertex_ptr vi = m_dcel->m_vertex[i];
                vi->setP(m_primitives_v[vi->idx].project(vi->P()));
            }
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::cleaning(HalfEdge_ptr he)
        {
            // let vs, vt, vl, vr be the vertices of the two triangles adjacent to he
            Vertex_ptr v[4];
            v[0] = he->V();                 //vs
            v[1] = he->Twin()->V();         //vt
            v[2] = he->Prev()->V();         //vl
            v[3] = he->Twin()->Prev()->V(); //vr

            // test if an edge vl-vr already exists
            // if so we have a "Y" configuration and we do not do the flip
            VHEIterator vlIt = VHEIterator(v[2]);
            HalfEdgeList adjHeVl = vlIt.list();
            for (uint i = 0; i < adjHeVl.size(); i++)
            {
                HalfEdge_ptr heVl = adjHeVl[i];
                if (heVl->Next()->V()->idx == v[3]->idx)
                {
                    return;
                }
            }

            // compute valence before flip
            FaceList vAdjFaces;
            Scalar deviation_pre = 0.0;
            VFIterator vFIt = VFIterator(v[0]);
            vAdjFaces = vFIt.list();
            deviation_pre += std::abs(Scalar(vAdjFaces.size()) - 6.0);
            if (vAdjFaces.size() < 3)
                return;
            for (uint i = 1; i < 4; i++)
            {
                vFIt = VFIterator(v[i]);
                vAdjFaces = vFIt.list();
                deviation_pre += std::abs(Scalar(vAdjFaces.size()) - 6.0);
                vAdjFaces.clear();
            }
            // compute normal before flip
            Vector3 n1_pre = Geometry::triangleNormal(he->V()->P(), he->Next()->V()->P(), he->Prev()->V()->P());
            Vector3 n2_pre = Geometry::triangleNormal(he->Twin()->V()->P(), he->Twin()->Next()->V()->P(), he->Twin()->Prev()->V()->P());
            // compute error before flip
            Primitive q;
            Vector3 p = Vector3::Zero();
            Scalar error_pre = m_em.computeError(he, m_primitives_v, p, q);

            // flip edge
            DcelOperations::flipEdge(*m_dcel, he->idx);
            // update primitive
            for (uint i = 0; i < 4; i++)
            {
                Primitive q;
                m_em.generateVertexPrimitive(q, v[i], m_scale, m_ring_size);
                m_primitives_v[v[i]->idx] = q;
            }

            // compute valence after flip
            Scalar deviation_post = 0.0;
            for (uint i = 0; i < 4; i++)
            {
                VFIterator vFIt = VFIterator(v[i]);
                vAdjFaces = vFIt.list();
                deviation_post += std::abs(Scalar(vAdjFaces.size()) - 6.0);
                vAdjFaces.clear();
            }
            // compute normal after flip
            Vector3 n1_post = Geometry::triangleNormal(he->V()->P(), he->Next()->V()->P(), he->Prev()->V()->P());
            Vector3 n2_post = Geometry::triangleNormal(he->Twin()->V()->P(), he->Twin()->Next()->V()->P(), he->Twin()->Prev()->V()->P());
            // compute error after flip
            p = Vector3::Zero();
            Scalar error_post = m_em.computeError(he, m_primitives_v, p, q);


            if ((deviation_pre <= deviation_post) ||
                    (n1_pre.dot(n1_post) < 0.0) ||
                    (n2_pre.dot(n2_post) < 0.0) ||
                    (error_pre <= error_post))
            {
                DcelOperations::flipEdge(*m_dcel, he->idx);
                // update primitive
                for (uint i = 0; i < 4; i++)
                {
                    Primitive q;
                    m_em.generateVertexPrimitive(q, v[i], m_scale, m_ring_size);
                    m_primitives_v[v[i]->idx] = q;
                }
            }
            else
            {
                LOG(logINFO) << "flip edge " << he->V()->idx << " " << he->Next()->V()->idx;
            }
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::updatePriorityQueue(PriorityQueue &pQueue, Index vsIndex, Index vtIndex, std::ofstream &file)
        {
            // we delete of the priority queue all the edge containing vs_id or vt_id
            pQueue.removeEdges(vsIndex);
            pQueue.removeEdges(vtIndex);

            double edgeError;
            Vector3 p = Vector3::Zero();

            VHEIterator vsHEIt = VHEIterator(m_dcel->m_vertex[vsIndex]);
            HalfEdgeList adjHE = vsHEIt.list();

            for (uint i = 0; i < adjHE.size(); i++)
            {
                HalfEdge_ptr he = adjHE[i];

                //  TODO : Interface ?

                //cleaning(he);
                if (he->V()->idx > he->Next()->V()->idx)
                {
                    he = he->Twin();
                }
                Primitive q;
                edgeError = m_em.computeError(he, m_primitives_v, p, q);
                m_primitives_he[he->idx] = q;

                pQueue.insert(PriorityQueue::PriorityQueueData(he->V()->idx, he->Next()->V()->idx, he->idx, he->F()->idx, edgeError, p));
            }
            //pQueue.display();
        }

        //--------------------------------------------------

        template <class ErrorMetric>
        bool ProgressiveMesh<ErrorMetric>::isEcolPossible(Index halfEdgeIndex, Vector3 pResult)
        {
            HalfEdge_ptr he = m_dcel->m_halfedge[halfEdgeIndex];

            // Look at configuration T inside a triangle
            bool hasTIntersection = false;
            VVIterator v1vIt = VVIterator(he->V());
            VVIterator v2vIt = VVIterator(he->Next()->V());
            VertexList adjVerticesV1 = v1vIt.list();
            VertexList adjVerticesV2 = v2vIt.list();

            uint countIntersection = 0;
            for (uint i = 0; i < adjVerticesV1.size(); i++)
            {
                for (uint j = 0; j < adjVerticesV2.size(); j++)
                {
                    if (adjVerticesV1[i]->idx == adjVerticesV2[j]->idx)
                        countIntersection++;
                }
            }
            if (countIntersection > 2)
            {
                //LOG(logINFO) << "The edge " << he->V()->idx << ", " << he->Next()->V()->idx << " in face " << he->F()->idx << " is not collapsable for now : T-Intersection";
                hasTIntersection = true;
                return false;
            }

            // Look if normals are consistents
            bool consitent = true;
            if (!isEcolConsistent(halfEdgeIndex, pResult))
                return false;

            // Look if normals of faces change after collapse
            bool isFlipped = false;
            EFIterator eIt = EFIterator(he);
            FaceList adjFaces = eIt.list();

            Index vsId = he->V()->idx;
            Index vtId = he->Next()->V()->idx;
            for (uint i = 0; i < adjFaces.size(); i++)
            {
                HalfEdge_ptr heCurr = adjFaces[i]->HE();
                Vertex_ptr v1 = nullptr;
                Vertex_ptr v2 = nullptr;
                Vertex_ptr v = nullptr;
                for (uint j = 0; j < 3; j++)
                {
                    if (heCurr->V()->idx != vsId && heCurr->V()->idx != vtId)
                    {
                        if (v1 == nullptr)
                            v1 = heCurr->V();
                        else if (v2 == nullptr)
                            v2 = heCurr->V();
                    }
                    else
                    {
                        v = heCurr->V();
                    }
                    heCurr = heCurr->Next();
                }
                if (v1 != nullptr && v2 != nullptr)
                {
                    Vector3 d1 = v1->P() - pResult;
                    Vector3 d2 = v2->P() - pResult;
                    d1.normalize();
                    d2.normalize();

                    //TEST
                    //Do we really need this ?

                    //Scalar a = fabs(d1.dot(d2));
                    //Vector3 d1_before = v1->P() - v->P();
                    //Vector3 d2_before = v2->P() - v->P();
                    //d1_before.normalize();
                    //d2_before.normalize();
                    //Scalar a_before = fabs(d1_before.dot(d2_before));
                    //if (a > 0.999 && a_before < 0.999)
                    //    isFlipped = true;

                    Vector3 fp_n = d1.cross(d2);
                    fp_n.normalize();
                    Vector3 f_n = Geometry::triangleNormal(v->P(), v1->P(), v2->P());
                    Scalar fpnDotFn = fp_n.dot(f_n);
                    if (fpnDotFn < 0.0) //-0.5
                    {
                        isFlipped = true;
                        //LOG(logINFO) << "The edge " << he->V()->idx << ", " << he->Next()->V()->idx << " in face " << he->F()->idx << " is not collapsable for now : Flipped face";
                        return false;
                        break;
                    }
                }
            }

            //return ((!hasTIntersection) && (consitent));
            return ((!hasTIntersection) && (!isFlipped) && (consitent));
            //return ((!hasTIntersection) && (!isFlipped));
            //return !hasTIntersection;
        }

        template <class ErrorMetric>
        bool ProgressiveMesh<ErrorMetric>::checkConsistency(FaceList adjFaces, Vertex_ptr v, Face_ptr f1, Face_ptr f2, Vector3 pResult, bool &consistent)
        {
            for (uint i = 0; i < adjFaces.size() && consistent; i++)
            {
                Face_ptr f = adjFaces[i];
                if ((f != f1) && (f != f2))
                {
                    HalfEdge_ptr h = f->HE();
                    Vertex_ptr vf = h->V(); //he->V()
                    while (vf != v)
                    {
                        h = h->Next();
                        vf = h->V();
                    }
                    h = h->Next();
                    Vertex_ptr vs = h->V();
                    Vertex_ptr vt = h->Next()->V();

                    Vector3 vsvt = vs->P() - vt->P();
                    Vector3 nf = Geometry::triangleNormal(v->P(), vs->P(), vt->P());
                    Vector3 n = vsvt.cross(nf);

                    consistent = ((n.dot(v->P()) >= 0.0) == (n.dot(pResult) >= 0.0));
                    if (! consistent)
                    {
                        return false;
                        //LOG(logINFO) << "Edge is not collapsable due to inconsistency";
                    }
                }
            }
            return consistent;
        }

        //Quadric-Based Polygonal Surface Simplification, PhD thesis by Michael Garland (1999), p.56-57 : Consistency Checks
        template <class ErrorMetric>
        bool ProgressiveMesh<ErrorMetric>::isEcolConsistent(Index halfEdgeIndex, Vector3 pResult)
        {
            HalfEdge_ptr he = m_dcel->m_halfedge[halfEdgeIndex];
            Face_ptr f1 = he->F();
            Face_ptr f2 = he->Twin()->F();
            Vertex_ptr v1 = he->V();
            Vertex_ptr v2 = he->Next()->V();

            VFIterator v1fIt = VFIterator(v1);
            VFIterator v2fIt = VFIterator(v2);
            FaceList adjFacesV1 = v1fIt.list();
            FaceList adjFacesV2 = v2fIt.list();

            bool consistent = true;

            checkConsistency(adjFacesV1, v1, f1, f2, pResult, consistent);
            checkConsistency(adjFacesV2, v2, f1, f2, pResult, consistent);

            return consistent;
        }

        //--------------------------------------------------

        template <class ErrorMetric>
        std::vector<ProgressiveMeshData> ProgressiveMesh<ErrorMetric>::constructM0(int targetNbFaces, int &nbNoFrVSplit, int primitiveUpdate, Scalar scale, int weightPerVertex, std::ofstream &file)
        {
            //uint nbPMData = 0;
            m_scale = scale;
            m_weight_per_vertex = weightPerVertex;
            m_primitive_update = primitiveUpdate;
            m_ring_size = std::floor((m_scale/m_mean_edge_length) + 1);
            LOG(logINFO) << "Ring Size = " << m_ring_size << "...";
            LOG(logINFO) << "Scale = " << m_scale << "...";

            std::vector<ProgressiveMeshData> pmdata;
            pmdata.reserve(targetNbFaces);

            LOG(logINFO) << "Computing Vertices Primitives...";
            computeVerticesPrimitives();

            LOG(logINFO) << "Computing Priority Queue...";
            PriorityQueue pQueue;
            constructPriorityQueue(pQueue);
            PriorityQueue::PriorityQueueData d;

            LOG(logINFO) << "Collapsing...";
            ProgressiveMeshData data;

            std::ofstream file2 ("error_pqueue_update.dat", std::ofstream::out);
            uint nb_faces_start = m_nb_faces;
            uint progression = 0;

            while (m_nb_faces > targetNbFaces)
            {
                if (std::abs(Scalar(Scalar((nb_faces_start - m_nb_faces)) / Scalar(nb_faces_start - targetNbFaces)) - progression * 0.10) < (1.0 / Scalar(nb_faces_start - targetNbFaces)))
                {
                    LOG(logINFO) << progression * 10 << "% done";
                    progression += 1;
                }

                if (pQueue.empty()) break;
                d = pQueue.top();

                HalfEdge_ptr he = m_dcel->m_halfedge[d.m_edge_id];

                // TODO !
                if (!isEcolPossible(he->idx, d.m_p_result))
                    continue;

                if (he->Twin() == nullptr)
                {
                    m_nb_faces -= 1;
                    nbNoFrVSplit++;
                }
                else
                {
                    m_nb_faces -= 2;
                }
                m_nb_vertices -= 1;

//#ifdef CORE_DEBUG
#ifdef ENABLE_DEBUG_CONTENT
                data.setError(d.m_err);
                data.setPResult(d.m_p_result);

                /*
                data.setQCenter(m_primitives_he[he->idx].center());
                data.setQRadius(m_primitives_he[he->idx].radius());
                data.setQ1Center(m_primitives_v[d.m_vs_id].center());
                data.setQ1Radius(m_primitives_v[d.m_vs_id].radius());
                data.setQ2Center(m_primitives_v[d.m_vt_id].center());
                data.setQ2Radius(m_primitives_v[d.m_vt_id].radius());
                data.setVs((m_dcel->m_vertex[d.m_vs_id])->P());
                data.setVt((m_dcel->m_vertex[d.m_vt_id])->P());
                data.setGradientQ1(m_primitives_v[d.m_vs_id].primitiveGradient(data.getVs()));
                data.setGradientQ2(m_primitives_v[d.m_vt_id].primitiveGradient(data.getVt()));
                    */
                /*
                std::vector<ProgressiveMeshData::DataPerEdgeColor> err_per_edge = pQueue.copyToVector(m_dcel->m_halfedge.size(), *m_dcel);
                data.setErrorPerEdge(err_per_edge);
                */
#endif
//#endif

                //
                //Vector3 v0 = m_dcel->m_vertex[d.m_vs_id]->P();
                //Vector3 v1 = m_dcel->m_vertex[d.m_vt_id]->P();
                //

                DcelOperations::edgeCollapse(*m_dcel, d.m_edge_id, d.m_p_result, true, data);

                updateVerticesPrimitives(d.m_vs_id, he);
                //updateVerticesPrimitives(d.m_vs_id, he, v0, v1, d.m_vs_id, d.m_vt_id, file);
                updatePriorityQueue(pQueue, d.m_vs_id, d.m_vt_id, file2);

                pmdata.push_back(data);

                //nbPMData++;
            }
            LOG(logINFO) << "Collapsing done";
            file2.close();

            return pmdata;
        }

        //--------------------------------------------------

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::vsplit(ProgressiveMeshData pmData)
        {
            HalfEdge_ptr he = m_dcel->m_halfedge[pmData.getHeFlId()];
            if (he->Twin() == NULL)
                m_nb_faces += 1;
            else
                m_nb_faces += 2;
            m_nb_vertices += 1;

            DcelOperations::vertexSplit(*m_dcel, pmData);
        }

        template <class ErrorMetric>
        void ProgressiveMesh<ErrorMetric>::ecol(ProgressiveMeshData pmData)
        {
            HalfEdge_ptr he = m_dcel->m_halfedge[pmData.getHeFlId()];
            if (he->Twin() == NULL)
                m_nb_faces -= 1;
            else
                m_nb_faces -= 2;
            m_nb_vertices -= 1;

            DcelOperations::edgeCollapse(*m_dcel, pmData);
        }
    }
}

