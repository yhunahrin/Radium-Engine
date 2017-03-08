#ifndef VERTEXOPERATION_H
#define VERTEXOPERATION_H


#include <Core/RaCore.hpp>

#include <Core/Math/LinearAlgebra.hpp>

#include <Core/Mesh/DCEL/Dcel.hpp>

#include <Core/Mesh/ProgressiveMesh/ProgressiveMeshData.hpp>

namespace Ra {
namespace Core {
namespace DcelOperations {

    /*! \brief Compute a vertex normal */
    RA_CORE_API Vector3 vertexNormal(Vertex_ptr vi);


    /*! \brief Create a vertex split from data pmdata */
    /*
    void createVt(Dcel& dcel, ProgressiveMeshData pmdata);
    void findFlclwNeig(Dcel& dcel, ProgressiveMeshData pmdata,
                       Index &flclwId, Index &flclwOpId, Index &frcrwId, Index &frcrwOpId,
                       FaceList adjOut);
    */
    RA_CORE_API void vertexSplit(Dcel& dcel, ProgressiveMeshData pmdata);
}
}
}

#endif // VERTEXOPERATION_H
