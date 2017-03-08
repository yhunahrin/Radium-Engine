#include <Core/RaCore.hpp>

#include <Core/Math/LinearAlgebra.hpp>

#include <Core/Mesh/DCEL/Dcel.hpp>
#include <Core/Mesh/ProgressiveMesh/ProgressiveMeshData.hpp>

namespace Ra {
namespace Core {
namespace DcelOperations {

    /*! \brief Split an edge */
    RA_CORE_API void splitEdge( Dcel& dcel, Index edgeIndex, Scalar fraction );

    /*! \brief Flip an edge */
    RA_CORE_API void flipEdge( Dcel& dcel, Index edgeIndex );

    /*! \brief Collapse an edge to pResult */
    RA_CORE_API void edgeCollapse( Dcel& dcel, Index edgeIndex, Vector3 pResult, bool updatePMData, ProgressiveMeshData& data);
    void edgeCollapse( Dcel& dcel, ProgressiveMeshData pmData);
}
}
}
