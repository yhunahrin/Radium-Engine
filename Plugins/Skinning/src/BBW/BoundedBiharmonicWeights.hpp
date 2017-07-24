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
    /// Options of the BBW computation
    struct BBWOptions
    {
        uint nBoneSamples {5};      /// Number of extra points sampled along the bones
        uint nBBWIter     {20};     /// Number of BBW solver iterations
        bool verbose      {false};  /// Output debug messages from tetgen and igl
        bool outputTetMesh{false};  /// If true, the tet mesh will be outputed for debugging
    };



    /// This function compute bounded biharmonic weights on a surface mesh with a skeleton.
    /// see "Bounded biharmonic Weights for Real-Time deformation", Jacobson et al. SIGGRAPH 2011.
    ///
    /// The computation of BBW relies on having a volumetric mesh (tetrahedral), and requires
    /// that some of the tet-mesh vertices are positionned precisely on the handles.
    /// In our case, this means that some tet-mesh points have to be on the bones segments,
    /// (but not on the joints).
    /// This function first compute the volumetric tetmesh using Tetgen, then calls
    /// the igl implementation of BBW.
    /// In case of failure it will return an empty matrix.
    WeightMatrix computeBBW( const Ra::Core::TriangleMesh& mesh, const Ra::Core::Animation::Skeleton& skel,
                             const BBWOptions& options = BBWOptions());

}
#endif

}


#endif //SKINNING_PLUGIN_BOUNDED_BIHARMONIC_WEIGHTS_HPP_
