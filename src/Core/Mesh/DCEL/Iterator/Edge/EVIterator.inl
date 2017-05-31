#include <Core/Mesh/DCEL/Iterator/Edge/EVIterator.hpp>

#include <Core/Mesh/DCEL/HalfEdge.hpp>
#include <Core/Mesh/DCEL/Face.hpp>

namespace Ra {
namespace Core {



/// CONSTRUCTOR
EVIterator::EVIterator( HalfEdge_ptr& he ) : EIterator< Vertex >( he )
{
}



/// DESTRUCTOR
EVIterator::~EVIterator() { }



/// LIST
inline VertexList EVIterator::list() const
{
    VertexList L;
    HalfEdge_ptr h1 = m_he;
    HalfEdge_ptr h2 = h1 != NULL ? h1->Twin() : NULL;
    Vertex_ptr v1 = h1->V();
    Vertex_ptr v2 = h2->V();

    // First path around v2
    do {
        if (h1 != NULL)
        {
            L.push_back(v1);
            if (v1 == v2) break;
            h1 = h1->Next()->Twin();
            v1 = h1->V();
        }
        if (h2 != NULL)
        {
            L.push_back(v2);
            if (v1 == v2) break;
            h2 = h2->Prev()->Twin();
            v2 = h2->Next()->V();
        }
    } while( h1 != NULL || h2 != NULL );

    // Second path around v1
    h1 = m_he;
    h2 = h1 != NULL ? h1->Twin()->Next()->Twin()->Next()->Twin() : NULL;
    h1 = h1->Prev()->Twin()->Prev()->Twin();
    v1 = h1->Next()->V();
    v2 = h2->V();
    do {
        if (h1 != NULL)
        {
            L.push_back(v1);
            if (v1 == v2) break;
            h1 = h1->Prev()->Twin();
            v1 = h1->Next()->V();
        }
        if (h2 != NULL)
        {
            L.push_back(v2);
            if (v1 == v2) break;
            h2 = h2->Next()->Twin();
            v2 = h2->V();
        }
    } while( h1 != NULL || h2 != NULL );

    return L;
}



/// OPERATOR
inline Vertex* EVIterator::operator->() const
{
    return m_he->V().get();
}



} // namespace Core
} // namespace Ra
