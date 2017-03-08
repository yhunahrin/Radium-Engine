#include <Core/Mesh/DCEL/Operations/EdgeOperation.hpp>
#include <Core/Mesh/DCEL/HalfEdge.hpp>
#include <Core/Mesh/DCEL/Vertex.hpp>
#include <Core/Mesh/DCEL/Vertex.hpp>
#include <Core/Mesh/DCEL/FullEdge.hpp>
#include <Core/Mesh/DCEL/Iterator/Vertex/VHEIterator.hpp>

namespace Ra {
namespace Core {
namespace DcelOperations {

    void splitEdge( Dcel& dcel, Index edgeIndex, Scalar fraction )
    {

        // Global schema of operation
        /*

         before                                          after

              A                                            A
            /   \                                     /    |    \
           /  F0 \                                   / F0  |  F2 \
          /       \                                 /      |      \
         / --he0-> \                               / he0-> | he2-> \
        V1--edge --V2                           V1  -----  M ------V2
         \ <-he1-- /                               \ <-he1 | <-he3 /
          \       /                                 \      |      /
           \  F1 /                                   \ F1  |  F3 /
            \   /                                     \    |    /
              B                                        \   B   /


        */

        CORE_ASSERT( fraction > 0 && fraction < 1, "Invalid fraction" );


        FullEdge_ptr edge = dcel.m_fulledge[edgeIndex];


        Vertex_ptr v1 = edge->V( 0 );
        Vertex_ptr v2 = edge->V( 1 );

        // step one : create the new data structures

        // 1) The edge we will split.

        // HalfEdges from the edge we will split.
        // They will stay twin as the "left part" of the split edge.
        HalfEdge_ptr he0 = edge->HE( 0 );
        HalfEdge_ptr he1 = he0->Twin();

        // Current edge becomes the "left part"( V1->M)
        FullEdge_ptr fe0 = edge;


        // The two new half edges for the "right part"
        HalfEdge_ptr he2( new HalfEdge() );
        HalfEdge_ptr he3( new HalfEdge() );

        // New edge as the "right part" (M->V2)
        FullEdge_ptr fe1( new FullEdge( he2 ) );

        // 2) New vertex M
        Vertex_ptr vm( new Vertex(
            fraction * v1->P() + (1. - fraction) * v2->P(),
            (fraction * v1->N() + (1. - fraction) * v2->N()).normalized(),
            he2 ) );


        // 3) The two new edges created by joining M with the opposed vertices.

        // Half edge joining M to A (on side of F0)
        HalfEdge_ptr heA0( new HalfEdge );
        // Half edge joining M to A (on side of F1)
        HalfEdge_ptr heA2( new HalfEdge );
        // Full edge MA
        FullEdge_ptr feA( new FullEdge( heA0 ) );

        // Half edge joining M to B (on side of F1)
        HalfEdge_ptr heB1( new HalfEdge );
        // Half edge joining M to B (on side of F3)
        HalfEdge_ptr heB3( new HalfEdge );
        // Full edge MB
        FullEdge_ptr feB( new FullEdge( heB1 ) );

        // 4 ) Two faces adjacent to the edge to split
        Face_ptr f0 = he0->F();
        Face_ptr f1 = he1->F();

        // Two new faces
        Face_ptr f2( new Face() );
        Face_ptr f3( new Face() );


        // Step two : update the data structure

        // Save all existing points and half edges for fixup

        Vertex_ptr A = he0->Prev()->V();
        Vertex_ptr B = he1->Prev()->V();

        HalfEdge_ptr AV1 = he0->Prev();
        HalfEdge_ptr V2A = AV1->Prev();

        HalfEdge_ptr V1B = he1->Next();
        HalfEdge_ptr BV2 = V1B->Next();

        // Insert new elements
        dcel.m_vertex.insert( vm , vm->idx );

        dcel.m_halfedge.insert( he2, he2->idx );
        dcel.m_halfedge.insert( he3, he3->idx );

        dcel.m_fulledge.insert( fe1, fe1->idx );

        dcel.m_face.insert( f2, f2->idx );
        dcel.m_face.insert( f3, f3->idx );

        // Fixup all pointers.

        he0->setNext( heA0 );
        he1->setPrev( heB1 );
        he1->setV( vm );

        he2->setV( vm );
        he2->setNext( V2A );
        he2->setPrev( heA2 );
        he2->setTwin( he3 );
        he2->setFE( fe1 );
        he2->setF( f2 );

        he3->setV( v2 );
        he3->setNext( heB3 );
        he3->setPrev( BV2 );
        he3->setTwin( he2 );
        he3->setFE( fe1 );
        he3->setF( f1 );

        heA0->setV( vm );
        heA0->setNext( AV1 );
        heA0->setPrev( he0 );
        heA0->setTwin( heA2 );
        heA0->setFE( feA );
        heA0->setF( f0 );

        heA2->setV( A );
        heA2->setNext( he2 );
        heA2->setPrev( V2A );
        heA2->setTwin( heA0 );
        heA2->setFE( feA );
        heA2->setF( f2 );

        heB1->setV( B );
        heB1->setNext( he1 );
        heB1->setPrev( V1B );
        heB1->setTwin( heB3 );
        heB1->setFE( feB );
        heB1->setF( f1 );

        heB3->setV( vm );
        heB3->setNext( BV2 );
        heB3->setPrev( he3 );
        heB3->setTwin( heB1 );
        heB3->setFE( feB );
        heB3->setF( f3 );


        if ( f0->HE() == V2A )
        {
            f0->HE() = heA0;
        }

        if ( f1->HE() == BV2 )
        {
            f1->HE() = heB1;
        }

        f2->HE() = he2;
        f3->HE() = he3;
    }

    //------------------------------------------------------------

    void flipEdge( Dcel& dcel, Index edgeIndex)
    {
        HalfEdge_ptr he_vsvt = dcel.m_halfedge[edgeIndex];
        HalfEdge_ptr he_vtvs = he_vsvt->Twin();

        Face_ptr f1 = he_vsvt->F();
        Face_ptr f2 = he_vtvs->F();

        Vertex_ptr vs = he_vsvt->V();
        Vertex_ptr vt = he_vtvs->V();
        Vertex_ptr vl = he_vsvt->Prev()->V();
        Vertex_ptr vr = he_vtvs->Prev()->V();

        HalfEdge_ptr he_vrvt = he_vtvs->Prev();
        HalfEdge_ptr he_vsvr = he_vtvs->Next();
        HalfEdge_ptr he_vtvl = he_vsvt->Next();
        HalfEdge_ptr he_vlvs = he_vsvt->Prev();

        HalfEdge_ptr he_vlvr = he_vsvt;
        HalfEdge_ptr he_vrvl = he_vtvs;

        he_vlvr->setV(vl);
        he_vrvl->setV(vr);

        he_vlvr->setNext(he_vrvt);
        he_vrvl->setNext(he_vlvs);

        he_vlvr->setPrev(he_vtvl);
        he_vrvl->setPrev(he_vsvr);

        he_vlvr->setTwin(he_vrvl);
        he_vrvl->setTwin(he_vlvr);

    }

    //------------------------------------------------------------

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

    void edgeCollapse(Dcel& dcel, Index edgeIndex, Vector3 pResult, bool updatePMData, ProgressiveMeshData& data)
    {
        CORE_ASSERT(dcel.m_halfedge[edgeIndex]->V()->idx != dcel.m_halfedge[edgeIndex]->Twin()->V()->idx,"Twins with same starting vertex.");

        //Exception
        /*
                    T
                   /|\
                  / | \
                 /  |  \
                /   S   \
               /   / \   \
              /   /   \   \
             /   /     \   \
            /   /       \   \
           V1 — — — — — — — V2

           We have to delete the 3 faces
        */


        // Retrieve the edge to collapse
        HalfEdge_ptr edge = dcel.m_halfedge[edgeIndex];

        Vertex_ptr v1 = edge->V();
        Vertex_ptr v2 = edge->Next()->V();

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

        // Make the halfEdge of the vertices of the faces
        // to delete point to existing new edges if needed
        if (h1->Prev()->V()->HE()->F() == f1)
            h1->Prev()->V()->setHE(h1->Next()->Twin());
        if (h2->Prev()->V()->HE()->F() == f2)
            h2->Prev()->V()->setHE(h2->Next()->Twin());
        if (v1->HE()->F() == f2)
            v1->setHE(v1->HE()->Twin()->Next());
        else if (v1->HE()->F() == f1)
            v1->setHE(v1->HE()->Prev()->Twin());

        //-----------------------------------------------
        // TODO do the same with full edges !!!
        //-----------------------------------------------
        // TODO do something for mesh with holes
        //-----------------------------------------------

        // Delete the faces
        f1->setHE(NULL);
        if (f2 != NULL) f2->setHE(NULL);

        VHEIterator vIt = VHEIterator(v2);
        HalfEdgeList adjHE = vIt.list();
        for (uint i = 0; i < adjHE.size(); i++)
        {
            adjHE[i]->setV(v1);
        }

        // Set new position of v1 and delete v2
        v1->setP(pResult);
        v2->setHE(NULL);

        // Updating twins
        HalfEdge_ptr e1 = (h1->Prev())->Twin();
        HalfEdge_ptr e2 = (h1->Next())->Twin();
        e1->setTwin(e2);
        e2->setTwin(e1);
        if (h2 != NULL)
        {
            HalfEdge_ptr e3 = (h2->Prev())->Twin();
            HalfEdge_ptr e4 = (h2->Next())->Twin();
            e3->setTwin(e4);
            e4->setTwin(e3);
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

    void edgeCollapse( Dcel& dcel, ProgressiveMeshData pmData)
    {
        // compute PResult
        Vector3 vtPos = dcel.m_vertex[pmData.getVtId()]->P();
        Vector3 vsPos = dcel.m_vertex[pmData.getVsId()]->P();
        Vector3 pResult = pmData.computePResult(vtPos, vsPos);

        edgeCollapse(dcel, pmData.getHeFlId(), pResult, false, pmData);
    }

}
}
}
