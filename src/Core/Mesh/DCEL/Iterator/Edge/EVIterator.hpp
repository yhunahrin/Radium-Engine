#ifndef RADIUMENGINE_DCEL_EDGE_VERTEX_ITERATOR_HPP
#define RADIUMENGINE_DCEL_EDGE_VERTEX_ITERATOR_HPP

#include <Core/Mesh/DCEL/Iterator/Edge/EdgeIterator.hpp>

namespace Ra {
namespace Core {

class EVIterator : public EIterator< Vertex > {
public:
    /// CONSTRUCTOR
    inline EVIterator( HalfEdge_ptr& he );
    inline EVIterator( const EVIterator& it ) = default;

    /// DESTRUCTOR
    inline ~EVIterator();

    /// LIST
    inline VertexList list() const override;

    /// OPERATOR
    inline Vertex* operator->() const override;
};

} // namespace Core
} // namespace Ra

#include <Core/Mesh/DCEL/Iterator/Edge/EVIterator.inl>

#endif // RADIUMENGINE_DCEL_EDGE_VERTEX_ITERATOR_HPP
