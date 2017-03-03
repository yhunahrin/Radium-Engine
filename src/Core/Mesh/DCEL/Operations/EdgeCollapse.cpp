#include <Core/Mesh/DCEL/Operations/EdgeCollapse.hpp>
#include <Core/Mesh/DCEL/HalfEdge.hpp>
#include <Core/Mesh/DCEL/Vertex.hpp>
#include <Core/Mesh/DCEL/FullEdge.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VHEIterator.hpp>
#include <Core/Log/Log.hpp>


namespace Ra {
    namespace Core {
        namespace DcelOperations {

            short int computeii(Dcel& dcel, Index vsId, Index vtId, Vector3 pResult, Vector3 &vadS, Vector3 &vadL)
            {
                Vector3 vtPos = dcel.m_vertex[vtId]->P();
                Vector3 vsPos = dcel.m_vertex[vsId]->P();
                Scalar disVtVp = (vtPos - pResult).squaredNorm();
                Scalar distVmVp = (((vtPos + vsPos) / 2.f) - pResult).squaredNorm();
                Scalar distVsVp = (vsPos - pResult).squaredNorm();
                short int ii = std::min(disVtVp, std::min(distVmVp, distVsVp));
                if (ii == 0)
                {
                    vadS = vtPos - pResult;
                    vadL = vsPos - pResult;
                }
                else if (ii == 1)
                {
                    vadS = vsPos - pResult;
                    vadL = vtPos - pResult;
                }
                else
                {
                    vadS = ((vtPos + vsPos) / 2.f) - pResult;
                    vadL = (vtPos - vsPos) / 2.f;
                }
                return ii;
            }

            //------------------------------------------------------------

            void edgeCollapse(Dcel& dcel, Index edgeIndex, Vector3 pResult, bool updatePMData, ProgressiveMeshData& data)
            {
                CORE_ASSERT(dcel.m_halfedge[edgeIndex]->V()->idx != dcel.m_halfedge[edgeIndex]->Twin()->V()->idx, "Twins with same starting vertex.");

                // Retrieve the edge to collapse and its vertices
                HalfEdge_ptr edge = dcel.m_halfedge[edgeIndex];
                Vertex_ptr v1 = edge->V();
                Vertex_ptr v2 = edge->Next()->V();

                /*
                if (data.getTConfig().hasTConfig())
                {
                    LOG(logINFO) << edge->V()->idx          << ", " << edge->F()->idx           << ", " << edge->Next()->V()->idx           << ", " << edge->Prev()->V()->idx;
                    LOG(logINFO) << edge->Twin()->V()->idx  << ", " << edge->Twin()->F()->idx   << ", " << edge->Twin()->Next()->V()->idx   << ", " << edge->Twin()->Prev()->V()->idx;

                    LOG(logINFO) << edge->Prev()->Twin()->V()->idx << ", " << edge->Prev()->Twin()->F()->idx << ", " << edge->Prev()->Twin()->Next()->V()->idx << ", " << edge->Prev()->Twin()->Prev()->V()->idx;
                    LOG(logINFO) << edge->Next()->Twin()->V()->idx << ", " << edge->Next()->Twin()->F()->idx << ", " << edge->Next()->Twin()->Next()->V()->idx << ", " << edge->Next()->Twin()->Prev()->V()->idx;

                    LOG(logINFO) << edge->Twin()->Prev()->Twin()->V()->idx << ", " << edge->Twin()->Prev()->Twin()->F()->idx << ", " << edge->Twin()->Prev()->Twin()->Next()->V()->idx << ", " << edge->Twin()->Prev()->Twin()->Prev()->V()->idx;
                    LOG(logINFO) << edge->Twin()->Next()->Twin()->V()->idx << ", " << edge->Twin()->Next()->Twin()->F()->idx << ", " << edge->Twin()->Next()->Twin()->Next()->V()->idx << ", " << edge->Twin()->Next()->Twin()->Prev()->V()->idx;
                }
                */

                // Retrieve the two halfedges
                HalfEdge_ptr h1 = edge;
                HalfEdge_ptr h2 = h1->Twin();

                // Retrieve the two faces
                Face_ptr f1 = h1->F();
                Face_ptr f2 = (h2 != NULL) ? h2->F() : nullptr;

                // Data for ProgressiveMeshData
                Vector3 vadS, vadL;
                short int ii = computeii(dcel, v1->idx, v2->idx, pResult, vadS, vadL);
                Vertex_ptr vl = h1->Prev()->V();
                Vertex_ptr vr = h2->Prev()->V();
                Face_ptr flclw = h1->Prev()->Twin()->F();
                ProgressiveMeshData::TConfiguration t = data.getTConfig();
                bool tconfinfl = (h1->Next())->Twin()->V()->idx == t.getVTConfigId() ? true : false;

                // Set Halfedge of vl and vr
                if (!t.hasTConfig() || (t.hasTConfig() && !tconfinfl))
                {
                    if (h1->Prev()->V()->HE()->F() == f1)
                    {
                        h1->Prev()->V()->setHE(h1->Next()->Twin());
                    }
                }
                if (!t.hasTConfig() || (t.hasTConfig() && tconfinfl))
                {
                    if (h2->Prev()->V()->HE()->F() == f2)
                    {
                        h2->Prev()->V()->setHE(h2->Next()->Twin());
                    }
                }

                // Set Halfedge of v1
                if (v1->HE()->F() == f2)
                {
                    v1->setHE(v1->HE()->Twin()->Next());
                }
                else if (v1->HE()->F() == f1)
                {
                    v1->setHE(v1->HE()->Prev()->Twin());
                }

                //-----------------------------------------------
                // TODO do the same with full edges !!!
                //-----------------------------------------------
                // TODO do something for mesh with holes
                //-----------------------------------------------

                // Delete the faces
                f1->setHE(NULL);
                if (f2 != NULL)
                {
                    f2->setHE(NULL);
                }

                VHEIterator vIt = VHEIterator(v2);
                HalfEdgeList adjHE = vIt.list();
                for (uint i = 0; i < adjHE.size(); i++)
                {
                    adjHE[i]->setV(v1);
                }

                // Set new position of v1 and delete v2
                v1->setP(pResult);
                v2->setHE(NULL);

                HalfEdge_ptr e1 = (h1->Prev())->Twin();
                HalfEdge_ptr e2 = (h1->Next())->Twin();

                if (!t.hasTConfig())
                {
                    // Updating twins
                    e1->setTwin(e2);
                    e2->setTwin(e1);
                    if (h2 != NULL)
                    {
                        HalfEdge_ptr e3 = (h2->Prev())->Twin();
                        HalfEdge_ptr e4 = (h2->Next())->Twin();
                        e3->setTwin(e4);
                        e4->setTwin(e3);
                    }
                }
                else // T configuration
                {
                    // Delete 2 faces and 1 vertex in a T configuation case
                    // Update twins
                    // Update vl (or vr)
                    if (tconfinfl)
                    {
                        if (h2 != NULL)
                        {
                            HalfEdge_ptr e3 = (h2->Prev())->Twin();
                            HalfEdge_ptr e4 = (h2->Next())->Twin();
                            e3->setTwin(e4);
                            e4->setTwin(e3);
                        }
                    }
                    else
                    {
                        e1->setTwin(e2);
                        e2->setTwin(e1);
                    }
                    dcel.m_face[t.getF1TConfigId()]->setHE(NULL);
                    dcel.m_face[t.getF2TConfigId()]->setHE(NULL);
                    HalfEdge_ptr et1 = tconfinfl ? e1->Prev()->Twin() : h2->Prev()->Twin()->Prev()->Twin();
                    HalfEdge_ptr et2 = tconfinfl ? e2->Next()->Twin() : h2->Next()->Twin()->Next()->Twin();
                    et1->setTwin(et2);
                    et2->setTwin(et1);
                    if (tconfinfl)
                    {
                        vl = et2->V();
                        if (vl->HE()->F()->idx == t.getF2TConfigId() || vl->HE()->F()->idx == t.getF1TConfigId())
                        {
                            vl->setHE(et2);
                        }
                    }
                    else
                    {
                        vr = et2->V();
                        if (vr->HE()->F()->idx == t.getF2TConfigId() || vr->HE()->F()->idx == t.getF1TConfigId())
                        {
                            vr->setHE(et2);
                        }
                    }
                    dcel.m_vertex[t.getVTConfigId()]->setHE(NULL);

                    if (v1->HE()->F()->idx == t.getF1TConfigId())
                    {
                        v1->setHE(v1->HE()->Prev()->Twin());
                    }
                    else if (v1->HE()->F()->idx == t.getF2TConfigId())
                    {
                        v1->setHE(v1->HE()->Twin()->Next());
                    }
                }

                // update ProgressiveMeshData on this edge collapse
                if (updatePMData)
                {
                    data.setVadl(vadL);
                    data.setVads(vadS);
                    data.setHeFlId(edgeIndex);
                    data.setHeFrId(dcel.m_halfedge[edgeIndex]->Twin()->idx);
                    data.setFlclwId(flclw->idx);
                    data.setFlId(f1->idx);
                    data.setFrId(f2->idx);
                    data.setVsId(v1->idx);
                    data.setVtId(v2->idx);
                    data.setVlId(vl->idx);
                    data.setVrId(vr->idx);
                    data.setii(ii);
                }
            }

            //------------------------------------------------------------

            void edgeCollapse( Dcel& dcel, ProgressiveMeshData pmData)
            {
                // compute PResult
                Vector3 vtPos = dcel.m_vertex[pmData.getVtId()]->P();
                Vector3 vsPos = dcel.m_vertex[pmData.getVsId()]->P();
                Vector3 pResult = pmData.computePResult(vtPos, vsPos);

                edgeCollapse(dcel, pmData.getHeFlId(), pResult, false, pmData);
            }


        } // Dcel Operations
    } // Core
} // Ra
