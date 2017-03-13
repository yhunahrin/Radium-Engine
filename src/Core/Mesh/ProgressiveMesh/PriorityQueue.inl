#include "PriorityQueue.hpp"
#include <Core/Log/Log.hpp>
#include <Core/Mesh/ProgressiveMesh/ProgressiveMeshData.hpp>
#include <Core/Mesh/DCEL/Vertex.hpp>

namespace Ra
{
    namespace Core
    {

        inline void PriorityQueue::insert(PriorityQueueData item)
        {
            PriorityQueueContainer::iterator it;
            for ( it = m_priority_queue.begin();
                  it != m_priority_queue.end();
                  it++)
            {
                PriorityQueue::PriorityQueueData data = *it;
                if (data.m_vs_id == item.m_vs_id && data.m_vt_id == item.m_vt_id)
                {
                    LOG(logINFO) << "PB here";
                    display();
                }
            }

            if (item.m_vs_id > item.m_vt_id)
            {
                LOG(logINFO) << "PB Insert in priority queue swapped";
                display();
            }
            CORE_ASSERT(item.m_vs_id < item.m_vt_id, "Insert in priority queue swapped");

            std::pair<PriorityQueueContainer::iterator,bool> pair = m_priority_queue.insert(item);
            if (!(bool)pair.second)
            {
                LOG(logINFO) << "PB Bad insert in priority queue";
                display();
            }
            CORE_ASSERT((bool)pair.second, "Bad insert in priority queue");

            std::pair<VertexHashContainer::iterator,bool> pair_vh = m_vertex_hash.insert(item);
            if (!(bool)pair_vh.second)
            {
                LOG(logINFO) << "PB Bad insert in vertex hash";
                display();
            }
            CORE_ASSERT((bool)pair_vh.second, "Bad insert in vertex hash");

            pair_vh = m_vertex_hash.insert(item.getSwapped());
            if (!(bool)pair_vh.second)
            {
                LOG(logINFO) << "PB Bad insert in vertex hash";
                display();
            }
            CORE_ASSERT((bool)pair_vh.second, "Bad insert in vertex hash");
        }

        //------------------------------

        inline PriorityQueue::PriorityQueueData PriorityQueue::top()
        {

            CORE_ASSERT(! m_priority_queue.empty(), "Invalid reference vertex");

            PriorityQueueContainer::iterator it_priority_queue;
            PriorityQueue::PriorityQueueData data;

            it_priority_queue = m_priority_queue.begin();
            data = *it_priority_queue;

            m_priority_queue.erase(it_priority_queue);
            m_vertex_hash.erase(m_vertex_hash.find(data));
            m_vertex_hash.erase(m_vertex_hash.find(data.getSwapped()));

            CORE_ASSERT(data.m_vs_id < data.m_vt_id, "Index has to be in order");
            return data;
        }

        inline void PriorityQueue::reserve(size_t size)
        {
           // m_vertex_hash.get_allocator().allocate(2*size);
           //m_priority_queue.get_allocator().allocate(size);
        }


        //------------------------------

        inline bool PriorityQueue::empty()
        {
            return size() <= 0;
        }

        //------------------------------

        inline int PriorityQueue::size()
        {
            return m_priority_queue.size();
        }

        //------------------------------

        /*
        inline std::vector<ProgressiveMeshData::DataPerEdgeColor> PriorityQueue::copyToVector(unsigned int nb_edges, Dcel& dcel)
        {
            std::vector<ProgressiveMeshData::DataPerEdgeColor> err_per_edge;
            err_per_edge.reserve(nb_edges + 2);
            for (unsigned int i = 0; i <= nb_edges; i++)
            {
                ProgressiveMeshData::DataPerEdgeColor d;
                err_per_edge.push_back(d);
            }
            PriorityQueue::PriorityQueueData data;
            Scalar error_max = -100000.0;
            Scalar error_min = 100000.0;
            for (PriorityQueueContainer::iterator it = m_priority_queue.begin(); it != m_priority_queue.end(); ++it)
            {
                data = *it;
                ProgressiveMeshData::DataPerEdgeColor d((dcel.m_vertex[data.m_vs_id])->P(), (dcel.m_vertex[data.m_vt_id])->P(), data.m_err);
                err_per_edge[data.m_edge_id] = d;
                if (data.m_err > error_max) error_max = data.m_err;
                if (data.m_err < error_min) error_min = data.m_err;
            }
            err_per_edge[nb_edges + 1].error = error_min;
            err_per_edge[nb_edges].error = error_max;
            return err_per_edge;
        }
        */

        inline std::vector<ProgressiveMeshData::DataPerEdgeColor> PriorityQueue::copyToVector(unsigned int nb_edges, Dcel& dcel)
        {
            std::vector<ProgressiveMeshData::DataPerEdgeColor> err_per_edge;
            err_per_edge.reserve(m_priority_queue.size() + 3);
            PriorityQueue::PriorityQueueData data;
            Scalar error_max = -100000.0;
            Scalar error_min = 100000.0;
            Scalar error_mediane = 0.0;
            unsigned int nb_data = 0;
            for (PriorityQueueContainer::iterator it = m_priority_queue.begin(); it != m_priority_queue.end(); ++it)
            {
                data = *it;
                ProgressiveMeshData::DataPerEdgeColor d((dcel.m_vertex[data.m_vs_id])->P(), (dcel.m_vertex[data.m_vt_id])->P(), data.m_err);
                err_per_edge.push_back(d);
                if (nb_data == int(m_priority_queue.size() / 2.0))
                    error_mediane = data.m_err;
                if (data.m_err > error_max) error_max = data.m_err;
                if (data.m_err < error_min) error_min = data.m_err;
                nb_data++;
            }
            ProgressiveMeshData::DataPerEdgeColor dmin(Vector3::Zero(), Vector3::Zero(), error_min);
            ProgressiveMeshData::DataPerEdgeColor dmax(Vector3::Zero(), Vector3::Zero(), error_max);
            ProgressiveMeshData::DataPerEdgeColor dmediane(Vector3::Zero(), Vector3::Zero(), error_mediane);
            err_per_edge.push_back(dmin);
            err_per_edge.push_back(dmax);
            err_per_edge.push_back(dmediane);
            return err_per_edge;
        }

    }

}
