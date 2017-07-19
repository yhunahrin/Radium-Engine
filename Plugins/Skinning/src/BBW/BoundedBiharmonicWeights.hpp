#ifndef SKINNING_PLUGIN_BOUNDED_BIHARMONIC_WEIGHTS_HPP_
#define SKINNING_PLUGIN_BOUNDED_BIHARMONIC_WEIGHTS_HPP_

#include <SkinningPluginMacros.hpp>
#include <Core/Math/LinearAlgebra.hpp>

namespace Ra { namespace Core { class TriangleMesh; }}
namespace Ra { namespace Core { namespace Animation { class Skeleton; }}}
typedef Ra::Core::Sparse WeightMatrix;

namespace SkinningPlugin
{

#if defined (SKINNING_WITH_BBW)
namespace BBW
{
    void computeBBW( const Ra::Core::TriangleMesh& mesh, const Ra::Core::Animation::Skeleton& skel, WeightMatrix& weightsOut);

}
#endif

}


#endif //SKINNING_PLUGIN_BOUNDED_BIHARMONIC_WEIGHTS_HPP_
