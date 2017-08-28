#include "Obb.hpp"

namespace Ra
{
    namespace Core
    {
        //
        // Bounding boxes functions.
        //

        template<uint D>
        inline typename ObbD<D>::AabbD ObbD<D>::toAabb() const
        {
            AabbD tmp;
            for ( int i = 0; i < 8; ++i )
            {
                tmp.extend( m_transform * m_aabb.corner( static_cast<Aabb::CornerType>( i ) ) );
            }
            return tmp;
        }

        template<uint D>
        inline typename ObbD<D>::Vector ObbD<D>::corner(int i) const
        {
            return m_aabb.corner(static_cast<typename ObbD<D>::AabbD::CornerType>(i));
        }

        template<uint D>
        inline typename ObbD<D>::Vector ObbD<D>::worldCorner( int i ) const
        {
            return m_transform * m_aabb.corner( static_cast<typename ObbD<D>::AabbD::CornerType>( i ) );
        }

        template<uint D>
        inline void ObbD<D>::addPoint( const typename ObbD<D>::Vector& p )
        {
            m_aabb.extend( p );
        }
    }
}
