#include <Engine/Managers/MeshContactManager/MeshContactManager.hpp>

#include <string>
#include <iostream>

#include <Core/Tasks/TaskQueue.hpp>

#include <Engine/RadiumEngine.hpp>
#include <Engine/Assets/FileData.hpp>
#include <Engine/Assets/HandleData.hpp>

#include <Engine/Entity/Entity.hpp>

#include "Eigen/Core"

#include <Core/Geometry/Normal/Normal.hpp>

#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>

namespace Ra
{
    namespace Engine
    {

        MeshContactManager::MeshContactManager()
            :m_nb_faces_max( 0 )
            ,m_nbfaces( 0 )
            ,m_threshold( 0.01 )
            ,m_lambda( 0.5 )
            ,m_curr_vsplit( 0 )
        {
        }

        void MeshContactManager::setNbFacesChanged(const int nb)
        {
            m_nbfaces = nb;
            computeNbFacesMax();
        }

        void MeshContactManager::computeNbFacesMax()
        {
            m_nb_faces_max = 0;

            for (const auto& elem : m_meshContactElements)
            {
                m_nb_faces_max += static_cast<MeshContactElement*>(elem)->getNbFacesMax();
            }
        }

        void MeshContactManager::setThresholdChanged(const double threshold)
        {
            m_threshold = std::pow(threshold,2);
        }

        void MeshContactManager::setLambdaChanged(const double lambda)
        {
            m_lambda = lambda;
        }

        void MeshContactManager::addMesh(MeshContactElement* mesh)
        {
            m_meshContactElements.push_back(mesh);
            //m_kdtrees.push_back(mesh->computeKdTree());
//            Super4PCS::KdTree<Scalar>* kdtree = new Super4PCS::KdTree<Scalar>();
//            m_kdtrees.push_back(kdtree);
//            m_kdtrees[m_kdtrees.size()-1] = mesh->computeKdTree();
//            LOG(logINFO) << "m_kdtrees size : " << m_kdtrees.size();
            //m_trianglekdtrees.push_back(mesh->computeTriangleKdTree());

            Super4PCS::TriangleKdTree<>* trianglekdtree = new Super4PCS::TriangleKdTree<>();
            m_trianglekdtrees.push_back(trianglekdtree);
            m_trianglekdtrees[m_trianglekdtrees.size()-1] = mesh->computeTriangleKdTree();
            LOG(logINFO) << "m_trianglekdtrees size : " << m_trianglekdtrees.size();

            mesh->computeProgressiveMesh();

            //Test of the closest triangle to an edge for a single object (cactus_256.obj)
//            bool b32 = false;
//            bool b77 = false;
//            int ind32, ind77;
//            int i = 0;
//            Ra::Core::IndexMap< Ra::Core::Vertex_ptr > vertices = mesh->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex;
//            int nbVertices = vertices.size();
//            while (! b32 || ! b77)
//            {
//                while (i < nbVertices)
//                {
//                    Ra::Core::Vector3 v = vertices[i]->P();
//                    if (v[0] >= 0.53 && v[0] <= 0.55 && v[1] >= 0.63 && v[1] <= 0.65 && v[2] >= 0.61 && v[2] <= 0.63)
//                    {
//                        b32 = true;
//                        ind32 = i;
//                    }
//                    else if (v[0] >= 0.53 && v[0] <= 0.55 && v[1] >= 0.70 && v[1] <= 0.72 && v[2] >= 0.70 && v[2] <= 0.72)
//                    {
//                        b77 = true;
//                        ind77 = i;
//                    }
//                    i++;
//                }
//            }
//            //int indFace = mesh->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContact(ind32,ind77,m_trianglekdtrees, m_trianglekdtrees.size()-1,m_threshold);
//            Ra::Core::Vector3 s1 = vertices[ind32]->P();
//            Ra::Core::Vector3 s2 = vertices[ind77]->P();
//            const Ra::Core::Vector3& segCenter = (Scalar)0.5 * (s1 + s2);
//            Ra::Core::Vector3 segDirection = s2 - s1;
//            Scalar segExtent = (Scalar)0.5 * std::sqrt((s2 - s1).dot(s2 - s1));
//            Ra::Core::IndexMap< Ra::Core::Face_ptr > triangles = mesh->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face;
//            int nbTriangles = triangles.size();
//            Scalar dist;
//            Scalar epsilon = Ra::Core::Math::dummyEps;
//            std::vector<int> facesAdj;
//            for (uint j = 0; j < nbTriangles; j++)
//            {
//                const Ra::Core::Vector3 triangle[3] = { triangles[j]->HE()->V()->P(),
//                                                        triangles[j]->HE()->Next()->V()->P(),
//                                                        triangles[j]->HE()->Prev()->V()->P() };
//                dist = Ra::Core::DistanceQueries::segmentToTriSq(segCenter,segDirection,segExtent,triangle).sqrDistance;
//                if (dist < epsilon)
//                {
//                    if (! (triangle[0] == s1 || triangle[0] == s2 || triangle[1] == s1 || triangle[1] == s2 || triangle[2] == s1 || triangle[2] == s2))
//                    {
//                        LOG(logINFO) << "ERROR test 1 on face " << j;
//                    }
//                    facesAdj.push_back(j);
//                }
//                else
//                {
//                    if (triangle[0] == s1 || triangle[0] == s2 || triangle[1] == s1 || triangle[1] == s2 || triangle[2] == s1 || triangle[2] == s2)
//                    {
//                        LOG(logINFO) << "ERROR test 2 on face " << j;
//                        LOG(logINFO) << "Face : " << triangle[0].transpose() << " --- " << triangle[1].transpose() << " --- " << triangle[2].transpose();
//                    }
//                }
//            }
//            std::vector<int> facesAdjKdtree;
//            mesh->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContacts(ind32,ind77,m_trianglekdtrees, m_trianglekdtrees.size()-1,epsilon, facesAdjKdtree);
//            std::sort(facesAdjKdtree.begin(), facesAdjKdtree.end());
//            for (std::vector<int>::iterator it=facesAdjKdtree.begin(); it!=facesAdjKdtree.end(); ++it)
//            {
//                LOG(logINFO) << ' ' << *it;
//            }
//            if (facesAdj == facesAdjKdtree)
//            {
//                LOG(logINFO) << "Faces found OK";
//            }
//            else
//            {
//                LOG(logINFO) << "Faces found NOT OK";
//            }

//            bool b46 = false;
//            bool b116 = false;
//            int ind46, ind116;
//            int i = 0;
//            Ra::Core::IndexMap< Ra::Core::Vertex_ptr > vertices = mesh->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex;
//            int nbVertices = vertices.size();
//            while (! b46 || ! b116)
//            {
//                while (i < nbVertices)
//                {
//                    Ra::Core::Vector3 v = vertices[i]->P();
//                    if (v[0] >= 0.49 && v[0] <= 0.51 && v[1] >= 0.43 && v[1] <= 0.45 && v[2] >= 0.83 && v[2] <= 0.85)
//                    {
//                        b46 = true;
//                        ind46 = i;
//                    }
//                    else if (v[0] >= 0.54 && v[0] <= 0.56 && v[1] >= 0.45 && v[1] <= 0.47 && v[2] >= 0.83 && v[2] <= 0.85)
//                    {
//                        b116 = true;
//                        ind116 = i;
//                    }
//                    i++;
//                }
//            }
//            int indFace = mesh->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContact(ind46,ind116,m_trianglekdtrees, m_trianglekdtrees.size()-1,m_threshold);

            mesh->getProgressiveMeshLOD()->getProgressiveMesh()->computeFacesQuadrics();
            //mesh->computePrimitives();
            mesh->computeFacePrimitives();
            computeNbFacesMax();
        }

        void MeshContactManager::setLodValueChanged(int value)
        {
            if (m_nbfaces < value)
            {
                while (m_nbfaces < value)
                {
                    MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[m_index_pmdata[--m_curr_vsplit]]);
                    int nbfaces = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces();
                    if (! obj->getProgressiveMeshLOD()->more())
                        break;
                    else
                    {
                        m_nbfaces += (obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces() - nbfaces);

                        //Find vs and vt in pmdata
    //                    int vsIndex = obj->getProgressiveMeshLOD()->getCurrentPMData().getVsId();
    //                    int vtIndex = obj->getProgressiveMeshLOD()->getCurrentPMData().getVtId();
    //                    obj->updateEllipsoidsVS(vsIndex,vtIndex);
                    }
                }
            }
            else if (m_nbfaces > value)
            {
                while (m_nbfaces > value)
                {
                    MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[m_index_pmdata[m_curr_vsplit++]]);
                    int nbfaces = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces();
                    if (! obj->getProgressiveMeshLOD()->less())
                        break;
                    else
                    {
                        m_nbfaces -= (nbfaces - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces());

                        //Find vs and vt in pmdata
    //                    int vsIndex = obj->getProgressiveMeshLOD()->getCurrentPMData().getVsId();
    //                    int vtIndex = obj->getProgressiveMeshLOD()->getCurrentPMData().getVtId();
    //                    obj->updateEllipsoidsEC(vsIndex,vtIndex);
                    }
                }
            }

            for (const auto& elem : m_meshContactElements)
            {
                  MeshContactElement* obj = static_cast<MeshContactElement*>(elem);
                  Ra::Core::TriangleMesh newMesh;
                  Ra::Core::convertPM(*(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()), newMesh);
                  obj->updateTriangleMesh(newMesh);

                  //obj->displayEllipsoids();

                  //add the display of the updated ellipsoids
            }
        }

        //simplification of the first loaded object only, the second one is there only to take into account contacts
        void MeshContactManager::setConstructM0()
        {
                     //constructPriorityQueues();
                    constructPriorityQueues2();

    //                  for (const auto& comp : m_components)
    //                  {
    //                        MeshContactComponent* obj = static_cast<MeshContactComponent*>(comp.second);
    //                        m_mainqueue.insert(obj->getPriorityQueue()->firstData());
    //                        LOG(logINFO) << "main queue size : " << m_mainqueue.size();
    //                  }

                     MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[0]);
                     m_mainqueue.insert(obj->getPriorityQueue()->firstData());


                // End criterion : number of faces set in the UI
                        int i = 0;

                        QueueContact::iterator it = m_mainqueue.begin();

                        while (it != m_mainqueue.end() && m_nb_faces_max > m_nbfaces)
                        {
                          const Ra::Core::PriorityQueue::PriorityQueueData &d = *it;
                          LOG(logINFO) << "Number of faces" << i << " : " << m_nb_faces_max;
                          MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[d.m_index]);
                          int nbfaces = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces();
                          int vs = d.m_vs_id;
                          int vt = d.m_vt_id;

                          //DEBUG for spinning top
                          if (vs == 190 || vt == 190)
                          {
                              LOG(logINFO) << "contact";
                          }

                          if (nbfaces > 2)
                          {
                            if (edgeCollapse(obj->getIndex()))
                            {
                              LOG(logINFO) << "Edge collapse of edge " << vs << " " << vt << " of object " << d.m_index << ", resulting vertex : (" << d.m_p_result(0,0) << ", " << d.m_p_result(1,0) << ", " << d.m_p_result(2,0) << ")";
                              m_index_pmdata.push_back(obj->getIndex());
                              m_curr_vsplit++;
                              m_nb_faces_max -= (nbfaces - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces());
                            }
                            LOG(logINFO) << "main queue size : " << m_mainqueue.size();
                            if (obj->getPriorityQueue()->size() > 0)
                            {
                                m_mainqueue.insert(obj->getPriorityQueue()->firstData());
                            }
                            else
                            {
                              LOG(logINFO) << "Priority queue empty";
                            }
                          }
                          LOG(logINFO) << "main queue size : " << m_mainqueue.size();
                          i++;
                          LOG(logINFO) << "i = " << i;
                          m_mainqueue.erase(it);
                          it = m_mainqueue.begin();
                        }

              for (const auto& elem : m_meshContactElements)
              {
                    MeshContactElement* obj = static_cast<MeshContactElement*>(elem);

                    //switch from DCEL to mesh
                    Ra::Core::TriangleMesh newMesh;
                    Ra::Core::convertPM(*(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()), newMesh);
                    obj->updateTriangleMesh(newMesh);

    //                obj->computeQuadricDisplay();

    //                obj->displayEllipsoids();
              }
        }

        int MeshContactManager::getNbFacesMax()
        {
            return m_nb_faces_max;
        }

        void MeshContactManager::constructPriorityQueues()
        {
            for (uint objIndex=0; objIndex < /*m_components.size()*/1; objIndex++)
            {
            MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[objIndex]);
            Ra::Core::PriorityQueue pQueue = Ra::Core::PriorityQueue();
            const uint numTriangles = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face.size();

            //browse edges
            Scalar  edgeError;
            Ra::Core::Vector3 p = Ra::Core::Vector3::Zero();
            int j;
            for (unsigned int i = 0; i < numTriangles; i++)
            {
                const Ra::Core::Face_ptr& f = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face.at( i );
                Ra::Core::HalfEdge_ptr h = f->HE();
                for (j = 0; j < 3; j++)
                {
                    const Ra::Core::Vertex_ptr& vs = h->V();
                    const Ra::Core::Vertex_ptr& vt = h->Next()->V();

                    // To prevent adding twice the same edge
                    if (vs->idx > vt->idx)
                    {
                        h = h->Next();
                        continue;
                    }

                    if (vs->idx == 190 || vt->idx == 190)
                    {
                       LOG(logINFO) << "CONTACT";
                    }


                    // test if the edge can be collapsed or if it has contact
                    int vertexIndex = -1;
                    bool contact = false;
                    Ra::Core::ProgressiveMesh<>::Primitive qk;
                    Scalar dist;
                    Scalar weight;
                    Scalar sumWeight = 0;

                    // for each vertex, we look for all contacts with other objects and add the contact quadrics to the quadric of the vertex
                    Ra::Core::ProgressiveMesh<>::Primitive qc = Ra::Core::ProgressiveMesh<>::Primitive();
                    for (uint k=0; k<m_kdtrees.size(); k++)
                    {
                        if (k != objIndex)
                        {
                            MeshContactElement* otherObj = static_cast<MeshContactElement*>(m_meshContactElements[k]);

                            vertexIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->vertexContact(vs->idx, m_kdtrees, k, m_threshold);
                            if ( vertexIndex != -1)
                            {
                                contact = true;
                                const Ra::Core::Vertex_ptr& c = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vertexIndex];
                                //dist = (c->P() - vs->P()).norm(); //in kdtree.hpp, the distance is a squared distance
                                dist = (c->P() - vs->P()).squaredNorm();
                                weight = std::pow(std::pow(dist/m_threshold,2)-1,2);
                                //weight = 1;
                                sumWeight += weight;
                                qk = otherObj->getPrimitive(vertexIndex);
                                qk *= weight;
                                qc += qk;
                            }

                            vertexIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->vertexContact(vt->idx, m_kdtrees, k, m_threshold);
                            if ( vertexIndex != -1)
                            {
                                contact = true;
                                const Ra::Core::Vertex_ptr& c = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vertexIndex];
                                //dist = (c->P() - vt->P()).norm();
                                dist = (c->P() - vt->P()).squaredNorm();
                                weight = std::pow(std::pow(dist/m_threshold,2)-1,2);
                                //weight = 1;
                                sumWeight += weight;
                                qk = otherObj->getPrimitive(vertexIndex);
                                qk *= weight;
                                qc += qk;
                            }
                        }
                    }

                    Ra::Core::ProgressiveMesh<>::Primitive qe = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeQuadric(h->idx);
                    Ra::Core::ProgressiveMesh<>::Primitive q = Ra::Core::ProgressiveMesh<>::Primitive(qe);
                    if (contact)
                    {
                        Ra::Core::EFIterator efIt = Ra::Core::EFIterator(h);
                        Ra::Core::FaceList facesAdj = efIt.list();
                        int nbFacesAdj = facesAdj.size();
                        q *= m_lambda * nbFacesAdj;
                        qc *= 1 - m_lambda;
                        q += qc;
                        q *= 1 / (m_lambda * nbFacesAdj + (1 - m_lambda) * sumWeight);
                    }

                    edgeError = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(h->idx, p, q);

                    //insert into the priority queue with the real resulting point
                    obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(h->idx, p, qe);
                    pQueue.insert(Ra::Core::PriorityQueue::PriorityQueueData(vs->idx, vt->idx, h->idx, i, edgeError, p, objIndex));

                    LOG(logINFO) << vs->idx << "   " << vt->idx << "   " << "error : " << edgeError;
                    h = h->Next();
                }
            }
            obj->setPriorityQueue(pQueue);
            }
        }

        void MeshContactManager::updatePriorityQueue(Ra::Core::Index vsIndex, Ra::Core::Index vtIndex, int objIndex)
        {
            MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[objIndex]);
            obj->getPriorityQueue()->removeEdges(vsIndex);
            obj->getPriorityQueue()->removeEdges(vtIndex);

            Scalar edgeError;
            Ra::Core::Vector3 p = Ra::Core::Vector3::Zero();
            Ra::Core::Index vIndex;

            //Listing of all the new edges formed with vs
            Ra::Core::VHEIterator vsHEIt = Ra::Core::VHEIterator(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vsIndex]);
            Ra::Core::HalfEdgeList adjHE = vsHEIt.list();

            // test if vs has any contacts
            int vertexIndex;
            bool contactVs = false;
            Ra::Core::ProgressiveMesh<>::Primitive qk;
            Scalar dist;
            Scalar weight;
            Scalar sumWeightVs = 0;

            Ra::Core::ProgressiveMesh<>::Primitive qVs = Ra::Core::ProgressiveMesh<>::Primitive();
            for (uint k=0; k<m_kdtrees.size(); k++)
            {
                if (k != objIndex)
                {
                    MeshContactElement* otherObj = static_cast<MeshContactElement*>(m_meshContactElements[k]);

                    vertexIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->vertexContact(vsIndex, m_kdtrees, k, m_threshold);
                    if ( vertexIndex != -1)
                    {
                        contactVs = true;
                        const Ra::Core::Vertex_ptr& c = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vertexIndex];
                        //dist = (c->P() - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vsIndex]->P()).norm();
                        dist = (c->P() - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vsIndex]->P()).squaredNorm();
                        weight = std::pow(std::pow(dist/m_threshold,2)-1,2);
                        //weight = 1;
                        sumWeightVs += weight;
                        qk = otherObj->getPrimitive(vertexIndex);
                        qk *= weight;
                        qVs += qk;
                    }
                }
            }

            // test if the other vertex of the edge has any contacts
            for (uint i = 0; i < adjHE.size(); i++)
            {
                Ra::Core::HalfEdge_ptr he = adjHE[i];

                int vIndex = he->Next()->V()->idx;

                bool contact = false;
                Scalar sumWeightV = 0;
                Ra::Core::ProgressiveMesh<>::Primitive qV = Ra::Core::ProgressiveMesh<>::Primitive();

                for (uint k=0; k<m_kdtrees.size(); k++)
                {
                    if (k != objIndex)
                    {
                        MeshContactElement* otherObj = static_cast<MeshContactElement*>(m_meshContactElements[k]);

                        vertexIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->vertexContact(vIndex, m_kdtrees, k, m_threshold);
                        if ( vertexIndex != -1)
                        {
                            contact = true;
                            const Ra::Core::Vertex_ptr& c = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vertexIndex];
                            //dist = (c->P() - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vIndex]->P()).norm();
                            dist = (c->P() - obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vIndex]->P()).squaredNorm();
                            weight = std::pow(std::pow(dist/m_threshold,2)-1,2);
                            //weight = 1;
                            sumWeightV += weight;
                            qk = otherObj->getPrimitive(vertexIndex);
                            qk *= weight;
                            qV += qk;
                        }
                    }
                }

                Ra::Core::ProgressiveMesh<>::Primitive qe = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeQuadric(he->idx);
                Ra::Core::ProgressiveMesh<>::Primitive qc = Ra::Core::ProgressiveMesh<>::Primitive();
                Ra::Core::ProgressiveMesh<>::Primitive q = Ra::Core::ProgressiveMesh<>::Primitive(qe);
                Scalar sumWeight = sumWeightVs + sumWeightV;
                if (contactVs)
                {
                    qc += qVs;
                }
                if (contact)
                {
                    qc += qV;
                }
                if (contactVs || contact)
                {
                    Ra::Core::EFIterator efIt = Ra::Core::EFIterator(he);
                    Ra::Core::FaceList facesAdj = efIt.list();
                    int nbFacesAdj = facesAdj.size();
                    q *= m_lambda * nbFacesAdj;
                    qc *= 1 - m_lambda;
                    q += qc;
                    q *= 1 / (m_lambda * nbFacesAdj + (1 - m_lambda) * sumWeight);
                }

                // compute the error while considering contacts in the quadric
                edgeError = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(he->idx, p, q);

                // insert into the priority queue with the real resulting point
                obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(he->idx, p, qe);

                // check that the index of the starting point of the edge is smaller than the index of its ending point
                if (vsIndex < vIndex)
                {
                    obj->getPriorityQueue()->insert(Ra::Core::PriorityQueue::PriorityQueueData(vsIndex, vIndex, he->idx, he->F()->idx, edgeError, p, objIndex));
                }
                else
                {
                    obj->getPriorityQueue()->insert(Ra::Core::PriorityQueue::PriorityQueueData(vIndex, vsIndex, he->Twin()->idx, he->Twin()->F()->idx, edgeError, p, objIndex));
                }
            }
        }

        void MeshContactManager::constructPriorityQueues2()
        {
            for (uint objIndex=0; objIndex < /*m_components.size()*/1; objIndex++)
            {
            MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[objIndex]);
            Ra::Core::PriorityQueue pQueue = Ra::Core::PriorityQueue();
            const uint numTriangles = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face.size();

            //browse edges
            Scalar  edgeError;
            Ra::Core::Vector3 p = Ra::Core::Vector3::Zero();
            int j;
            for (unsigned int i = 0; i < numTriangles; i++)
            {
                const Ra::Core::Face_ptr& f = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face.at( i );
                Ra::Core::HalfEdge_ptr h = f->HE();
                for (j = 0; j < 3; j++)
                {
                    const Ra::Core::Vertex_ptr& vs = h->V();
                    const Ra::Core::Vertex_ptr& vt = h->Next()->V();

                    // To prevent adding twice the same edge
                    if (vs->idx > vt->idx)
                    {
                        h = h->Next();
                        continue;
                    }

                    // test if the edge can be collapsed or if it has contact
                    //int faceIndex = -1;
                    std::vector<int> faceIndexes;
                    bool contact = false;
                    Ra::Core::ProgressiveMesh<>::Primitive qk;
                    Scalar sqdist;
                    Scalar weight;
                    int nbContacts = 0;

                    // for each edge, we look for all contacts with other objects and add the contact quadrics to the quadric of the edge
                    Ra::Core::ProgressiveMesh<>::Primitive qc = Ra::Core::ProgressiveMesh<>::Primitive();
                    for (uint k=0; k<m_trianglekdtrees.size(); k++)
                    {
                        if (k != objIndex)
                        {
                            MeshContactElement* otherObj = static_cast<MeshContactElement*>(m_meshContactElements[k]);

                            // Closest face
//                            faceIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContact(vs->idx, vt->idx, m_trianglekdtrees, k, m_threshold);
//                            if ( faceIndex != -1)
//                            {
//                                contact = true;
//                                const Ra::Core::Face_ptr& f = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face[faceIndex];
//                                /*Eigen::Matrix<Scalar, 3, 1>*/ Ra::Core::Vector3 triangle[3] = {f->HE()->V()->P(), f->HE()->Next()->V()->P(), f->HE()->Prev()->V()->P()};
//                                //dist = (c->P() - vs->P()).norm(); //in kdtree.hpp, the distance is a squared distance
//                                const Ra::Core::Vector3& segCenter = (Scalar)0.5 * (vs->P() + vt->P());
//                                const Ra::Core::Vector3& segDirection = vt->P() - vs->P();
//                                Scalar segExtent = (Scalar)0.5 * std::sqrt((vt->P() - vs->P()).dot(vt->P() - vs->P()));
//                                sqdist = Ra::Core::DistanceQueries::segmentToTriSq(segCenter, segDirection, segExtent, triangle).sqrDistance;
//                                weight = std::pow(std::pow(sqdist/m_threshold,2)-1,2);
//                                //weight = 1;
//                                sumWeight += weight;
//                                qk = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getFacesQuadrics()[faceIndex];
//                                qk *= weight;
//                                qc += qk;
//                            }

                            // All close faces
                            obj->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContacts(vs->idx, vt->idx, m_trianglekdtrees, k, m_threshold, faceIndexes);
                            if ( faceIndexes.size() != 0)
                            {
                                contact = true;
                                const Ra::Core::Vector3& segCenter = (Scalar)0.5 * (vs->P() + vt->P());
                                const Ra::Core::Vector3& segDirection = vt->P() - vs->P();
                                Scalar segExtent = (Scalar)0.5 * std::sqrt((vt->P() - vs->P()).dot(vt->P() - vs->P()));
                                for (uint l = 0; l < faceIndexes.size(); l++)
                                {
                                    const Ra::Core::Face_ptr& f = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face[faceIndexes[l]];
                                    Ra::Core::Vector3 triangle[3] = {f->HE()->V()->P(), f->HE()->Next()->V()->P(), f->HE()->Prev()->V()->P()};
                                    sqdist = Ra::Core::DistanceQueries::segmentToTriSq(segCenter, segDirection, segExtent, triangle).sqrDistance;
                                    weight = std::pow((sqdist/m_threshold) - 1.0 ,2);
                                    nbContacts++;
                                    //qk = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getFacesQuadrics()[faceIndexes[l]]
                                    qk = otherObj->getFacePrimitive(faceIndexes[l]);
                                    qk *= weight;
                                    qc += qk;
                                }
                            }
                        }
                    }

                    Ra::Core::ProgressiveMesh<>::Primitive qe = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeQuadric(h->idx);
                    Ra::Core::ProgressiveMesh<>::Primitive q = Ra::Core::ProgressiveMesh<>::Primitive(qe);
                    if (contact)
                    {
                        qc *= 1.0 / nbContacts;
                        qc *= m_lambda;
                        q += qc;
                    }

                    // computing the optimal placement for the resulting vertex
                    Scalar edgeErrorQEM = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeError(h->idx, p);
                    edgeError = abs(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getEM().computeGeometricError(q,p));

                    // test to compare contact error below with classic error
                    Scalar epsilon = Ra::Core::Math::dummyEps;
                    if (edgeError + epsilon < edgeErrorQEM)
                    {
                        LOG(logINFO) << "Contacts lower the error at halfedge " << h->idx;
                    }

                    //insert into the priority queue with the real resulting point
                    //obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(h->idx, p, qe);
                    pQueue.insert(Ra::Core::PriorityQueue::PriorityQueueData(vs->idx, vt->idx, h->idx, i, edgeError, p, objIndex));

                    LOG(logINFO) << vs->idx << "   " << vt->idx << "   " << "error : " << edgeError;
                    h = h->Next();
                }
            }
            obj->setPriorityQueue(pQueue);
            }
        }

        void MeshContactManager::updatePriorityQueue2(Ra::Core::Index vsIndex, Ra::Core::Index vtIndex, int objIndex)
        {
            MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[objIndex]);
            obj->getPriorityQueue()->removeEdges(vsIndex);
            obj->getPriorityQueue()->removeEdges(vtIndex);

            Scalar edgeError;
            Ra::Core::Vector3 p = Ra::Core::Vector3::Zero();

            //Listing of all the new edges formed with vs
            Ra::Core::VHEIterator vsHEIt = Ra::Core::VHEIterator(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_vertex[vsIndex]);
            Ra::Core::HalfEdgeList adjHE = vsHEIt.list();

            // test if the other vertex of the edge has any contacts
            for (uint i = 0; i < adjHE.size(); i++)
            {
                Ra::Core::HalfEdge_ptr h = adjHE[i];

                const Ra::Core::Vertex_ptr& vs = h->V();
                const Ra::Core::Vertex_ptr& vt = h->Next()->V();

                // test if the edge can be collapsed or if it has contact
                //int faceIndex = -1;
                std::vector<int> faceIndexes;
                bool contact = false;
                Ra::Core::ProgressiveMesh<>::Primitive qk;
                Scalar sqdist;
                Scalar weight;
                int nbContacts = 0;

                // for each edge, we look for all contacts with other objects and add the contact quadrics to the quadric of the edge
                Ra::Core::ProgressiveMesh<>::Primitive qc = Ra::Core::ProgressiveMesh<>::Primitive();
                for (uint k=0; k<m_trianglekdtrees.size(); k++)
                {
                    if (k != objIndex)
                    {
                        MeshContactElement* otherObj = static_cast<MeshContactElement*>(m_meshContactElements[k]);

                        // Closest face
//                        faceIndex = obj->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContact(vs->idx, vt->idx, m_trianglekdtrees, k, m_threshold);
//                        if ( faceIndex != -1)
//                        {
//                            contact = true;
//                            const Ra::Core::Face_ptr& f = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face[faceIndex];
//                            /*Eigen::Matrix<Scalar, 3, 1>*/ Ra::Core::Vector3 triangle[3] = {f->HE()->V()->P(), f->HE()->Next()->V()->P(), f->HE()->Prev()->V()->P()};
//                            //dist = (c->P() - vs->P()).norm(); //in kdtree.hpp, the distance is a squared distance
//                            const Ra::Core::Vector3& segCenter = (Scalar)0.5 * (vs->P() + vt->P());
//                            const Ra::Core::Vector3& segDirection = vt->P() - vs->P();
//                            Scalar segExtent = (Scalar)0.5 * std::sqrt((vt->P() - vs->P()).dot(vt->P() - vs->P()));
//                            sqdist = Ra::Core::DistanceQueries::segmentToTriSq(segCenter, segDirection, segExtent, triangle).sqrDistance;
//                            weight = std::pow(std::pow(sqdist/m_threshold,2)-1,2);
//                            //weight = 1;
//                            sumWeight += weight;
//                            qk = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getFacesQuadrics()[faceIndex];
//                            qk *= weight;
//                            qc += qk;
//                        }

                          // All close faces
                        obj->getProgressiveMeshLOD()->getProgressiveMesh()->edgeContacts(vs->idx, vt->idx, m_trianglekdtrees, k, m_threshold, faceIndexes);
                        if ( faceIndexes.size() != 0)
                        {
                            contact = true;
                            const Ra::Core::Vector3& segCenter = (Scalar)0.5 * (vs->P() + vt->P());
                            const Ra::Core::Vector3& segDirection = vt->P() - vs->P();
                            Scalar segExtent = (Scalar)0.5 * std::sqrt((vt->P() - vs->P()).dot(vt->P() - vs->P()));
                            for (uint l = 0; l < faceIndexes.size(); l++)
                            {
                                const Ra::Core::Face_ptr& f = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_face[faceIndexes[l]];
                                Ra::Core::Vector3 triangle[3] = {f->HE()->V()->P(), f->HE()->Next()->V()->P(), f->HE()->Prev()->V()->P()};
                                sqdist = Ra::Core::DistanceQueries::segmentToTriSq(segCenter, segDirection, segExtent, triangle).sqrDistance;
                                weight = std::pow((sqdist/m_threshold) - 1.0 ,2);
                                nbContacts++;
                                //qk = otherObj->getProgressiveMeshLOD()->getProgressiveMesh()->getFacesQuadrics()[faceIndexes[l]];
                                qk = otherObj->getFacePrimitive(faceIndexes[l]);
                                qk *= weight;
                                qc += qk;
                            }
                        }
                    }
                }

                Ra::Core::ProgressiveMesh<>::Primitive qe = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeQuadric(h->idx);
                Ra::Core::ProgressiveMesh<>::Primitive q = Ra::Core::ProgressiveMesh<>::Primitive(qe);
                if (contact)
                {
                    qc *= 1.0 / nbContacts;
                    qc *= m_lambda;
                    q += qc;
                }

                // computing the optimal placement for the resulting vertex
                Scalar edgeErrorQEM = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeError(h->idx, p);
                edgeError = abs(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getEM().computeGeometricError(q,p));

                // test to compare contact error below with classic error
                Scalar epsilon = Ra::Core::Math::dummyEps;
                if (edgeError + epsilon < edgeErrorQEM)
                {
                    LOG(logINFO) << "Contacts lower the error at halfedge " << h->idx;
                }

                // insert into the priority queue with the real resulting point
                //obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeEdgeErrorContact(h->idx, p, qe);

                // check that the index of the starting point of the edge is smaller than the index of its ending point
                if (vs->idx < vt->idx)
                {
                    obj->getPriorityQueue()->insert(Ra::Core::PriorityQueue::PriorityQueueData(vs->idx, vt->idx, h->idx, h->F()->idx, edgeError, p, objIndex));
                }
                else
                {
                    obj->getPriorityQueue()->insert(Ra::Core::PriorityQueue::PriorityQueueData(vt->idx, vs->idx, h->Twin()->idx, h->Twin()->F()->idx, edgeError, p, objIndex));
                }
            }
        }

        bool MeshContactManager::edgeCollapse(int objIndex)
        {
            MeshContactElement* obj = static_cast<MeshContactElement*>(m_meshContactElements[objIndex]);

            if (obj->isConstructM0())
            {
                //edge collapse and putting the collapse data in the ProgressiveMeshLOD
                Ra::Core::PriorityQueue::PriorityQueueData d = obj->getPriorityQueue()->top();
                Ra::Core::HalfEdge_ptr he = obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()->m_halfedge[d.m_edge_id];

                //retrieve the quadric of vt to store it into data
                Ra::Core::Vertex_ptr vt = he->Next()->V();
                Ra::Core::Quadric<3> qVt = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeVertexQuadric(vt->idx);
    //            Ra::Core::ProgressiveMesh<>::Primitive qVt = obj->getProgressiveMeshLOD()->getProgressiveMesh()->computeVertexQuadric(vt->idx);

                if (he->Twin() == nullptr)
                {
                    obj->getProgressiveMeshLOD()->getProgressiveMesh()->collapseFace();
                    obj->getProgressiveMeshLOD()->oneVertexSplitPossible();
                }
                else
                {
                    obj->getProgressiveMeshLOD()->getProgressiveMesh()->collapseFace();
                    obj->getProgressiveMeshLOD()->getProgressiveMesh()->collapseFace();
                }
                obj->getProgressiveMeshLOD()->getProgressiveMesh()->collapseVertex();
                Ra::Core::ProgressiveMeshData data = Ra::Core::DcelOperations::edgeCollapse(*(obj->getProgressiveMeshLOD()->getProgressiveMesh()->getDcel()), d.m_edge_id, d.m_p_result);

                //adding the quadric of vt to data
                data.setQVt(qVt);

                if (obj->getProgressiveMeshLOD()->getProgressiveMesh()->getNbFaces() > 0)
                {
                obj->getProgressiveMeshLOD()->getProgressiveMesh()->updateFacesQuadrics(d.m_vs_id);
                }
                //update the priority queue of the object
                //updatePriorityQueue(d.m_vs_id, d.m_vt_id, objIndex);
                updatePriorityQueue2(d.m_vs_id, d.m_vt_id, objIndex);
    //            else
    //            {
    //                while (obj->getPriorityQueue()->size() > 0)
    //                    obj->getPriorityQueue()->top();
    //            }
                obj->getProgressiveMeshLOD()->addData(data);
                obj->getProgressiveMeshLOD()->oneEdgeCollapseDone();

                return true;
            }
            else
            {
                return false;
            }
        }
    }
}
