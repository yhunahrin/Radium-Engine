#include <Core/File/FileUtils.hpp>

#include <numeric>
#include <algorithm>

namespace Ra
{
    namespace Asset
    {

        namespace
        {
            void compute_points_ordering( int parent,
                                          uint current,
                                          const HandleData* skel,
                                          const Ra::Core::AlignedStdVector< Ra::Core::Vector2i >& edgeList,
                                          const Core::Vector3Array &vertices,
                                          const std::vector<std::vector<uint>> &bmap,
                                          std::vector<uint> &vmap,
                                          uint &last )
            {
                // compute reordering w.r.t. current bone
                std::vector<uint> new_bmap = bmap[ current ];
                const Core::Vector3 P0 = skel->getComponent( current ).m_frame.translation();
                if (parent != -1) // has parent => proj vertex on bone
                {
                    const Core::Vector3 P1 = skel->getComponent( parent ).m_frame.translation();
                    const Core::Vector3 V = P0 - P1;
                    std::sort( new_bmap.begin(), new_bmap.end(),
                               [vertices,&P0,&P1,&V](uint a, uint b){ return (vertices[a]-P1).dot( V ) < (vertices[b]-P1).dot( V ); } );
                }
                else // is root => dist to joint
                {
                    std::sort( new_bmap.begin(), new_bmap.end(),
                               [vertices,&P0](uint a, uint b){ return (vertices[a]-P0).squaredNorm() < (vertices[b]-P0).squaredNorm(); } );
                }
                // final reordering w.r.t. skeleton hierarchy
                for (uint j=0; j<new_bmap.size(); ++j)
                {
                    vmap[ new_bmap[j] ] = last++;
                }
                for( const auto& edge : edgeList ) {
                    if( edge[0] == current ) {
                        compute_points_ordering( current, edge[1], skel, edgeList, vertices, bmap, vmap, last );
                    }
                }
            }

            template< typename ArrayT >
            void apply_reorder(ArrayT &v, const std::vector<uint> &order)
            {
                ArrayT temp( v.size() );
                #pragma omp parallel for
                for (uint i=0; i<v.size(); ++i)
                {
                    temp[ order[i] ] = v[i];
                }
                std::swap( v, temp );
            }
        }

        void reoder_points_per_bone( FileData* fdata )
        {
            if (!fdata->hasGeometry() || !fdata->hasHandle())
            {
                return;
            }

            std::vector< GeometryData* > geomData = fdata->getGeometryData();
            std::vector<   HandleData* > skelData = fdata->getHandleData();

            for ( HandleData* skel : skelData )
            {
                if (!skel->isSkeleton())
                {
                    continue;
                }

                // get the geometry corresponding to the current skeleton
                auto g = std::find_if( geomData.begin(), geomData.end(),
                                       [skel](const GeometryData* geom){return skel->getName() == geom->getName();} );
                if (g == geomData.end())
                {
                    LOG(logWARNING) << "Found skeleton with no corresponding geometry." << std::endl;
                    continue;
                }
                GeometryData* geom = *g;

                // get the max weighted bone for each vertex
                const uint nb_v = geom->getVerticesSize();
                std::vector<uint> bmap(nb_v, -1); // bmap[old vertex index] = max weight bone index
                std::vector<Scalar> wmax(nb_v, -std::numeric_limits<Scalar>::max());
                const uint nb_b = skel->getComponentDataSize();
                for (uint i=0; i<nb_b; ++i)
                {
                    const auto& ws = skel->getComponent( i ).m_weight;
                    #pragma omp parallel for
                    for( uint j = 0; j<ws.size(); ++j )
                    {
                        if( wmax[ ws[j].first ] < ws[j].second )
                        {
                            bmap[ ws[j].first ] = i;
                            wmax[ ws[j].first ] = ws[j].second;
                        }
                    }
                }
                wmax.clear();

                // sort vertices per max weighted bone
                std::vector<std::vector<uint>> old_bmap(nb_b+1); // old_bmap[bone index] = old vertices indices
                for (uint i=0; i<nb_v; ++i)
                {
                    if (bmap[i] == -1)
                    {
                        LOG(logWARNING) << "Found a vertex without bone weight" << std::endl;
                        old_bmap[nb_b].push_back( i );
                    }
                    else
                    {
                        old_bmap[ bmap[i] ].push_back( i );
                    }
                }
                bmap.clear();

                // compute vertices reordering w.r.t. bones hierarchy
                auto vertices = geom->getVertices();
                std::vector<uint> vmap(nb_v, -1); // vmap[old vertex index] = new vertex index
                auto edgeList = skel->getEdgeData();
                std::set< uint > root;
                for( uint i = 0; i < nb_b; ++i )
                {
                    root.insert( i );
                }
                for( const auto &e : edgeList )
                {
                    root.erase( e[1] );
                }
                uint last = 0;
                for( const auto &r : root )
                {
                    compute_points_ordering( -1, r, skel, edgeList, vertices, old_bmap, vmap, last);
                }

                // just put vertices without bones at the end
                for (uint j=0; j<old_bmap[nb_b].size(); ++j)
                {
                    vmap[ old_bmap[nb_b][j] ] = last++;
                }
                if (last != nb_v)
                {
                    std::cerr << "Error: " << last << " != " << nb_v << std::endl;
                }

                // apply vertices reordering to skel and geom data
                auto comps = skel->getComponentData();
                #pragma omp parallel for
                for (uint i = 0; i<comps.size(); ++i)
                {
                    auto& comp = comps[i];
                    for (auto& w : comp.m_weight)
                    {
                        w.first = vmap[ w.first ];
                    }
                }
                skel->setComponents( comps );

                apply_reorder( vertices, vmap );
                geom->setVertices( vertices );
                auto edges = geom->getEdges();
                #pragma omp parallel for
                for (uint i = 0; i<edges.size(); ++i)
                {
                    edges[i][0] = vmap[ edges[i][0] ];
                    edges[i][1] = vmap[ edges[i][1] ];
                }
                geom->setEdges( edges );
                auto faces = geom->getFaces();
                #pragma omp parallel for
                for (uint i = 0; i<faces.size(); ++i)
                {
                    for (uint j=0; j<faces[i].size(); ++j)
                    {
                        faces[i][j] = vmap[ faces[i][j] ];
                    }
                }
                geom->setFaces( faces );
                auto polyhedra = geom->getPolyhedra();
                #pragma omp parallel for
                for (uint i = 0; i<polyhedra.size(); ++i)
                {
                    for (uint j=0; j<polyhedra[i].size(); ++j)
                    {
                        polyhedra[i][j] = vmap[ polyhedra[i][j] ];
                    }
                }
                geom->setPolyhedra( polyhedra );
                auto normals = geom->getNormals();
                apply_reorder( normals, vmap );
                geom->setNormals( normals );
                auto tangents = geom->getTangents();
                apply_reorder( tangents, vmap );
                geom->setTangents( tangents );
                auto bitangents = geom->getBiTangents();
                apply_reorder( bitangents, vmap );
                geom->setBitangents( bitangents );
                auto texcoords = geom->getTexCoords();
                apply_reorder( texcoords, vmap );
                geom->setTexCoords( texcoords );
                auto colors = geom->getColors();
                apply_reorder( colors, vmap );
                geom->setColors( colors );
                auto weights = geom->getWeights();
                apply_reorder( weights, vmap );
                geom->setWeights( weights );
            }
        }
    }
}

