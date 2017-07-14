// Application to

#include <string>

#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Mesh/TriangleMesh.hpp>
#include <Core/Animation/Handle/Skeleton.hpp>
#include <Core/Animation/Handle/SkeletonUtils.hpp>

#include <Engine/Assets/FileData.hpp>
#include <Engine/Assets/HandleToSkeleton.hpp>

#include <tetgen.h>

#ifdef DEBUG
#   define VERBOSE
#endif
#include <igl/boundary_conditions.h>
#undef VERBOSE
#include <igl/bbw.h>

using namespace Ra;

void print_usage()
{
    std::cout<<"Usage : bbwcomputer <rigged_mesh_file>"<<std::endl;
    std::cout<<"Input file must have a mesh and a skeleton (Assimp input formats only)."<<std::endl;
    std::cout<<"If there are several meshes or skeletons, the first one of each will be used."<<std::endl;
}

// This function compute bounded biharmonic weights on a surface mesh with a skeleton.
// see "Bounded biharmonic Weights for Real-Time deformation", Jacobson et al. SIGGRAPH 2011.

// The computation of BBW relies on having a volumetric mesh (tetrahedral), and requires
// that some of the tet-mesh vertices are positionned precisely on the handles.
// In our case, this means that some tet-mesh points have to be on the bones segments,
// (but not on the joints).

// This function first compute the volumetric tetmesh using Tetgen, then calls
// the igl implementation of BBW.


int main(int argc, char**argv)
{
    // options
    const bool verbose = true;
    const bool outputTetMesh = false;
    const uint nBoneSamples = 5;

    if (argc < 2)
    {
        print_usage();
        exit(0);
    }

    // Load data
    // =========

    // Load the file given as argument. First mesh and first skeleton will be taken as input.
    std::string filename(argv[1]);

    Asset::FileData fileData(filename, verbose);

    auto geomData = fileData.getGeometryData();
    auto skelData = fileData.getHandleData();

    if (geomData.empty() || skelData.empty())
    {
        std::cerr<<"Error : file must contain mesh and skeleton"<<std::endl;
        exit(1);
    }

    std::map<uint,uint> indexTable;
    Core::Animation::Skeleton skel;
    Asset::createSkeleton(*skelData[0], skel, indexTable);

    Core::TriangleMesh mesh;
    mesh.m_vertices = geomData[0]->getVertices();
    mesh.m_normals = geomData[0]->getNormals();

    for (const auto& face : geomData[0]->getFaces() )
    {
        mesh.m_triangles.push_back(face.head<3>());
    }

    Core::Vector3Array boneSamples;

    // Sample points on the skeleton bones
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
            return 0;
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
        for (const auto& t : skel.getPose(Core::Animation::Handle::SpaceType::MODEL))
        {
            C(i,0) = t.translation()[0];
            C(i,1) = t.translation()[1];
            C(i,2) = t.translation()[2];
            ++i;
        }

        auto edges = skel.m_graph.getEdges();
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
        bbw_data.active_set_params.max_iter = 10;
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

    return 0;
}
