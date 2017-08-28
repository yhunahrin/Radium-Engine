#include "PriorityQueue.hpp"
#include <Core/Log/Log.hpp>
#include <iostream>

namespace Ra
{
    namespace Core
    {

        //------------------------------


        void PriorityQueue::removeEdges(int v_id)
        {
            PriorityQueue::PriorityQueueData comparator;
            comparator.m_vs_id = v_id;
            comparator.m_vt_id = -1;

            if (m_priority_queue.size() * 2.0 != m_vertex_hash.size())
            {
                LOG(logINFO) << "PB Priority queue and vertex hash not good";
                display();
            }
            CORE_ASSERT(m_priority_queue.size() * 2.0 == m_vertex_hash.size(), "Priority queue and vertex hash not good");

            VertexHashContainer::iterator it;
            for ( it = m_vertex_hash.find(comparator);
                  it != m_vertex_hash.end();
                  it = m_vertex_hash.find(comparator))
            {
                PriorityQueue::PriorityQueueData data = *it;
                m_vertex_hash.erase(it);
                if (data.m_vs_id > data.m_vt_id)
                {
                    m_priority_queue.erase(m_priority_queue.find(data.getSwapped()));
                }
                else
                {
                    m_priority_queue.erase(m_priority_queue.find(data));
                }
                m_vertex_hash.erase(m_vertex_hash.find(data.getSwapped()));
            }

            if (m_priority_queue.size() * 2.0 != m_vertex_hash.size())
            {
                LOG(logINFO) << "PB Priority queue and vertex hash not good";
                display();
            }
            CORE_ASSERT(m_priority_queue.size() * 2.0 == m_vertex_hash.size(), "Priority queue and vertex hash not good");

        }

        //------------------------------

        void PriorityQueue::display()
        {
            std::cout << "__________________" << std::endl;
            for(const auto& el : m_priority_queue)
            {
                LOG(logINFO) << "fl_id = "  << (el.m_fl_id).getValue()
                             << ", vs_id = "   << (el.m_vs_id).getValue()
                             << ", vt_id = "   << (el.m_vt_id).getValue()
                             << " et error = " << el.m_err;
                //LOG(logINFO) << el.m_err;
            }

            std::cout << "__________________" << std::endl;
            for(const auto& el : m_vertex_hash)
            {
                LOG(logINFO) << "fl_id = "  << (el.m_fl_id).getValue()
                             << ", vs_id = "   << (el.m_vs_id).getValue()
                             << ", vt_id = "   << (el.m_vt_id).getValue()
                             << " et error = " << el.m_err;
                //LOG(logINFO) << el.m_err;
            }
        }
    } // namespace Core
} // namespace Ra
