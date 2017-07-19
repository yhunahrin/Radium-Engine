#if defined (SKINNING_WITH_BBW)
#include <BBW/BoundedBiharmonicWeights.hpp>

#include <Core/Mesh/TriangleMesh.hpp>
#include <Core/Animation/Handle/Skeleton.hpp>
#include <Core/Animation/Handle/HandleWeight.hpp>
#include <Core/Animation/Handle/HandleWeightOperation.hpp>
#include <Core/Animation/Handle/SkeletonUtils.hpp>


#include <tetgen.h>
#ifdef DEBUG
#   define VERBOSE
#endif
#include <igl/boundary_conditions.h>
#undef VERBOSE
#include <igl/bbw.h>

typedef Ra::Core::Animation::Handle::SpaceType SpaceType;

using namespace Ra;

namespace SkinningPlugin
{
namespace BBW
{

void computeBBW(const Ra::Core::TriangleMesh& mesh, const Ra::Core::Animation::Skeleton& skel, Ra::Core::Animation::WeightMatrix& weightsOut)
{
    // options
    const bool verbose = true;
    const bool outputTetMesh = false;
    const uint nBoneSamples = 5;


    // Sample points on the skeleton bones (but not at the joints)
    Core::Vector3Array boneSamples;
    for (uint j = 0; j < skel.size(); ++j)
    {
        Core::Vector3 a, b;
        Core::Animation::SkeletonUtils::getBonePoints(skel, j, a ,b);

        for (uint i = 1; i <nBoneSamples; ++i)
        {
            Scalar t = Scalar(i) / Scalar(nBoneSamples);
            boneSamples.push_back( (1-t) * a + t *b);
        }
    }

    // PART 1
    // create tet mesh from mesh + skeleton
    // -------------------------------------------

    // Matrix types filled by the tet mesh algorithm.
    Eigen::MatrixXd V; // vertices
    Eigen::MatrixXi F; // faces
    Eigen::MatrixXi T; // tetrahedra

    // Launch tetgen
    {
        tetgenio input;
        tetgenio output;

        const uint numverts = mesh.m_vertices.size();
        const uint numextrapts = boneSamples.size();
        const uint numtris = mesh.m_triangles.size();

        if (verbose)
        {
            std::cout<<"### TET MESH GENERATION ###"<<std::endl;
            std::cout<<numverts<<" vertices | "<<numtris<<" triangles"<<std::endl;
            std::cout<<numextrapts<<" extra points ("<<numverts<<" - "<<numverts+numextrapts - 1<<")"<<std::endl;
        }

        // Copy mesh data to tetgen input struct.
        // Note : tetgen automatically deletes the allocated arrays using delete[]
        // see tetgenio::deinitialize()
        REAL* points = new REAL[3*(numverts+numextrapts)];
        for (uint i = 0 ; i < numverts; ++i)
        {
            const auto& v = mesh.m_vertices[i];
            points[3*i + 0] = v[0];
            points[3*i + 1] = v[1];
            points[3*i + 2] = v[2];
        }
        // Add the additional bone samples.
        for (uint i = 0 ; i < numextrapts; ++i)
        {
            const auto& v = boneSamples[i];
            points[3*numverts + 3*i + 0] = v[0];
            points[3*numverts + 3*i + 1] = v[1];
            points[3*numverts + 3*i + 2] = v[2];
        }

        input.pointlist = points;
        input.numberofpoints = numverts+numextrapts;

        // Create one tetget `facet` per triangle
        tetgenio::facet*  facets = new tetgenio::facet[numtris];
        for (uint i = 0; i < numtris; ++i)
        {
            const auto&t = mesh.m_triangles[i];
            int* indices = new int[3];
            indices[0] = t[0];
            indices[1] = t[1];
            indices[2] = t[2];

            tetgenio::polygon* faces = new tetgenio::polygon[1];

            faces[0].vertexlist = indices;
            faces[0].numberofvertices = 3;

            facets[i].numberofpolygons = 1;
            facets[i].polygonlist = faces;
            facets[i].holelist = nullptr;
            facets[i].numberofholes = 0;
        }


        input.facetlist = facets;
        input.numberoffacets = numtris;

        // tetgen command line flags
        std::string flags;
        flags +="Y"; // preserve input mesh
        flags +="p"; // use PLC
        flags +="q"; // refine mesh
        if (verbose){flags +="V";} // verbosity
        if (outputTetMesh){ flags +="g";} // export .mesh file

        // Run tetmesh.
        if (outputTetMesh)
        {
            tetrahedralize(flags.c_str(), &input, NULL);
            return;
        }
        else
        {
            tetrahedralize(flags.c_str(),&input, &output);
        }

        // Export output data
        V.resize(output.numberofpoints,3);
        F.resize(output.numberoftrifaces,3);
        T.resize(output.numberoftetrahedra,4);

        if(verbose)
        {
            std::cout<<"### Generated tet mesh ###"<<std::endl;
            std::cout<<output.numberofpoints<<" vertices | "<<output.numberoftrifaces<<" triangles | "<<output.numberoftetrahedra<<" tetrahedra"<<std::endl;
        }

        for (uint i = 0; i < output.numberofpoints; ++i)
        {
            for (uint j = 0; j < 3; ++j)
            {
                V(i,j) = output.pointlist[3*i+j];
            }
        }

        for (uint i = 0; i< output.numberoftrifaces; ++i)
        {
            for (uint j = 0; j < 3; ++j)
            {
                F(i,j) = output.trifacelist[3*i+j];
            }
        }

        for (uint i = 0; i< output.numberoftetrahedra; ++i)
        {

            for (uint j = 0; j < 4; ++j)
            {
                T(i,j) = output.tetrahedronlist[4*i+j];
            }
        }
    }

    // PART 2
    // Compute weights with igl BBW algorithm
    // =======================================

    Eigen::MatrixXd W; // Weights matrix
    {

        Eigen::MatrixXd C( skel.m_graph.size(), 3); // Handle position ( joints )
        uint i = 0;
        for (const auto& t : skel.getPose(SpaceType::MODEL))
        {
            C(i,0) = t.translation()[0];
            C(i,1) = t.translation()[1];
            C(i,2) = t.translation()[2];
            ++i;
        }

        const auto edges = skel.m_graph.getEdges();
        Eigen::MatrixXi E( edges.size(), 2); // Edges of skeleton graph
        for (uint i = 0; i < edges.size(); ++i)
        {
            E(i,0) = edges[i].first;
            E(i,1) = edges[i].second;
        }

        Eigen::VectorXi b;
        Eigen::MatrixXd bc;

        // Automatically create the boundary condition matrices
        // telling which vertices are on the bones and have a fixed weight of 1
        bool result = igl::boundary_conditions(V,T,C,Eigen::VectorXi(), E, Eigen::MatrixXi(),b,bc);

        if ( !result )
        {
            std::cout<<"Boundary condition error"<<std::endl;
            exit(1);
        }

        igl::BBWData bbw_data;
        bbw_data.active_set_params.max_iter = 50;
        bbw_data.verbosity = verbose ? 2 : 0;

        // Do BBW
        result = igl::bbw(V,T,b,bc,bbw_data, W);

        if ( !result )
        {
            std::cout<<"bbw error "<<std::endl;
            exit(1);
        }

        // Normalize the weights.
        igl::normalize_row_sums(W,W);
    }


    // Convert the weights into our sparse matrix representation

    weightsOut.resize(mesh.m_vertices.size(), skel.size());
    auto edges = skel.m_graph.getEdges();
    for (uint i = 0; i < mesh.m_vertices.size(); ++i)
    {
        for (uint c = 0; c < W.cols();++c)
        {
            if (W(i,c) > Ra::Core::Math::dummyEps )
            {
                uint j = edges[c].first;
                weightsOut.coeffRef(i,j)  = W(i,c);
            }
        }

    }
    Ra::Core::Animation::checkWeightMatrix(weightsOut, true);
}

}
}
#endif // SKINNING_WITH_BBW
