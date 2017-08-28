#ifndef RADIUM_OBB_HPP_
#define RADIUM_OBB_HPP_

#include <Core/RaCore.hpp>

#include <Core/Math/LinearAlgebra.hpp>
namespace Ra
{
    namespace Core
    {
        /// An oriented bounding box.
        template<uint D>
        class ObbD
        {
        public:
            typedef Eigen::AlignedBox<Scalar, D> AabbD;
            typedef Eigen::Transform<Scalar, D,Eigen::Affine> TransformD;
            typedef Eigen::Matrix<Scalar, D,1> Vector;

            /// Constructors and destructor.

            /// Initializes an empty bounding box.
            ObbD()
                : m_aabb(),  m_transform( TransformD::Identity() ) {}

            /// Initialize an OBB from an AABB and a transform.
            ObbD( const AabbD& aabb, const TransformD& tr )
                : m_aabb( aabb ), m_transform( tr ) {}

            /// Default copy constructor and assignment operator.
            ObbD( const ObbD& other ) = default;
            ObbD& operator=( const ObbD& other ) = default;

            ~ObbD() {}

            /// Return the AABB enclosing this
            AabbD toAabb() const;

            /// Extends the OBB with an new point.
            void addPoint( const Vector& p );

            /// Returns the position of the i^th corner of AABB (model space)
            Vector corner(int i) const;

            /// Returns the position of the ith corner of the OBB ( world space )
            Vector worldCorner( int i ) const;

        public:
            /// The untransformed AABB
            AabbD m_aabb;
            /// Orientation of the box.
            TransformD m_transform;
        };

        typedef ObbD<3> Obb;
    }
}
#include <Core/Math/Obb.inl>

#endif // RADIUM_OBB_HPP_

