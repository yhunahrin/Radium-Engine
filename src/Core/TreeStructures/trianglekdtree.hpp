#ifndef TRIANGLEKDTREE_H
#define TRIANGLEKDTREE_H

#include "bbox.hpp"

#include "Eigen/Core"

#include "Core/Mesh/TriangleMesh.hpp"

#include "Core/Geometry/Triangle/TriangleOperation.hpp"

#include <limits>
#include <iostream>
#include <numeric>

// max depth of the tree
#define KD_MAX_DEPTH 32

// number of neighbors
#define KD_TRIANGLES_PER_CELL 64

//namespace Super4PCS{

//template<typename _Scalar, typename _Index = int >
//class TriangleKdTree
//{
//public:

//    struct KdNode
//    {
//        union {
//            struct {
//                float splitValue; //value of the dim coordinate
//                unsigned int firstChildId:24;
//                unsigned int dim:2; //axis of the split
//                unsigned int leaf:1;
//            };
//            struct { // if leaf
//                std::vector<_Index> triangleIndices;
//            };
//        };
//    };

//    typedef _Scalar Scalar;
//    typedef _Index  Index;

//    static constexpr Index invalidIndex() { return -1; }

//    typedef Eigen::Matrix<Scalar,3,1> VectorType;
//    typedef Ra::Core::Triangle Triangle;
//    typedef AABB3D<Scalar> AxisAlignedBoxType;

//    typedef std::vector<KdNode>     NodeList;
//    typedef std::vector<VectorType> PointList;
//    typedef std::vector<Triangle>   TriangleList;
//    typedef std::vector<Index>      IndexList;

//    inline const NodeList&      _getNodes      (void) { return mNodes;      }
//    inline const PointList&     _getPoints     (void) { return mPoints;     }
//    inline const TriangleList&  _getTriangles  (void) { return mTriangles;  }
//    inline const IndexList&     _getIndices    (void) { return mIndices;    }

//    TriangleKdTree(const TriangleList& t, const PointList& p,
//           unsigned int nofTrianglesPerCell = KD_TRIANGLES_PER_CELL,
//           unsigned int maxDepth = KD_MAX_DEPTH );

//    inline void finalize();

//    inline const AxisAlignedBoxType& aabb() const  {return mAABB; }

//    ~TriangleKdTree();

//    inline void split(unsigned int nodeId, IndexList& triangleIndices, unsigned int dim, Scalar splitValue);

//    void createTree(unsigned int nodeId,
//                    IndexList& triangleIndices,
//                    unsigned int level,
//                    unsigned int targetCellsize,
//                    unsigned int targetMaxDepth);

//protected:
//    NodeList      mNodes;
//    PointList     mPoints;
//    TriangleList  mTriangles;
//    IndexList     mIndices;

//    AxisAlignedBoxType mAABB;
//    unsigned int _nofTrianglesPerCell;
//    unsigned int _maxDepth;
//};


//template<typename Scalar, typename Index>
//TriangleKdTree<Scalar, Index>::TriangleKdTree(const TriangleList& t, const PointList& p,
//                       unsigned int nofTrianglesPerCell,
//                       unsigned int maxDepth)
//    : mPoints(p),
//      mTriangles(t),
//      mIndices(mTriangles.size()),
//      mAABB(p.cbegin(), p.cend()),
//      _nofTrianglesPerCell(nofTrianglesPerCell),
//      _maxDepth(maxDepth)
//{
//    std::iota (mIndices.begin(), mIndices.end(), 0); // Fill with 0, 1, ..., nbTriangles-1.
//    finalize();
//}

//template<typename Scalar, typename Index>
//void TriangleKdTree<Scalar, Index>::finalize()
//{
//    mNodes.clear();
//    mNodes.reserve(4*mTriangles.size()/_nofTrianglesPerCell);
//    mNodes.push_back(KdNode());
//    mNodes.back().leaf = 0;
//    std::cout << "create tree" << std::endl;
//    createTree(0, mIndices, 1, _nofTrianglesPerCell, _maxDepth);
//    std::cout << "create tree ... DONE (" << mTriangles.size() << " triangles)" << std::endl;
//}

//template<typename Scalar, typename Index>
//TriangleKdTree<Scalar, Index>::~TriangleKdTree()
//{
//}

//template<typename Scalar, typename Index>
//void TriangleKdTree<Scalar, Index>::split(unsigned int nodeId, IndexList& triangleIndices, unsigned int dim, Scalar splitValue) //equal number of triangles in both sides are needed
//{
//    for (unsigned int i = 0; i < triangleIndices.size(); i++)
//    {
//        if (mPoints[mTriangles[triangleIndices[i]][0]][dim] < splitValue || mPoints[mTriangles[triangleIndices[i]][1]][dim] < splitValue || mPoints[mTriangles[triangleIndices[i]][2]][dim] < splitValue)
//        {
//            if (mPoints[mTriangles[triangleIndices[i]][0]][dim] < splitValue && mPoints[mTriangles[triangleIndices[i]][1]][dim] < splitValue && mPoints[mTriangles[triangleIndices[i]][2]][dim] < splitValue)
//            {
//                mNodes[mNodes[nodeId].firstChildId].triangleIndices.pushback(triangleIndices[i]);
//            }
//            else
//            {
//                mNodes[mNodes[nodeId].firstChildId].triangleIndices.pushback(triangleIndices[i]);
//                mNodes[mNodes[nodeId].firstChildId+1].triangleIndices.pushback(triangleIndices[i]);
//            }
//        }
//        else
//        {
//            if (mPoints[mTriangles[triangleIndices[i]][0]][dim] >= splitValue && mPoints[mTriangles[triangleIndices[i]][1]][dim] >= splitValue && mPoints[mTriangles[mIndices[i]][2]][dim] >= splitValue)
//            {
//                mNodes[mNodes[nodeId].firstChildId+1].triangleIndices.pushback(triangleIndices[i]);
//            }
//            else
//            {
//                mNodes[mNodes[nodeId].firstChildId+1].triangleIndices.pushback(triangleIndices[i]);
//                mNodes[mNodes[nodeId].firstChildId].triangleIndices.pushback(triangleIndices[i]);
//            }
//        }
//    }
//}

//template<typename Scalar, typename Index>
//void TriangleKdTree<Scalar, Index>::createTree(unsigned int nodeId, TriangleKdTree<Scalar, Index>::IndexList& triangleIndices, unsigned int level, unsigned int targetCellSize, unsigned int targetMaxDepth)
//{
//    KdNode& node = mNodes[nodeId];
//    AxisAlignedBoxType aabb;

//    std::cout << "level : " << level << std::endl;

//    PointList AABBPoints; //to compute the aabbox
//    for (unsigned int i = 0; i < triangleIndices.size(); i++)
//    {
//        for (unsigned int j = 0; j < 3; j++)
//        {
//            AABBPoints.push_back(mPoints[mTriangles[triangleIndices[i]][j]]);
//        }
//    }
//    for (unsigned int k = 0; k< AABBPoints.size(); k++)
//    {
//        aabb.extendTo(AABBPoints[k]);
//    }

//    VectorType diag =  Scalar(0.5) * (aabb.max()- aabb.min()); //middle of the aabbox
//    typename VectorType::Index dim;

//    diag.maxCoeff(&dim); // finding the longest axis

//    node.dim = dim;
//    node.splitValue = aabb.center()(dim); // spliting the longest axis

//    split(nodeId, triangleIndices, dim, node.splitValue); //sorting the triangles in regard to the split value of the selected axis (left and right)

//    node.firstChildId = mNodes.size();

//    {
//        KdNode n;
//        n.size = 0;
//        mNodes.push_back(n);
//        mNodes.push_back(n);
//    }
//    //mNodes << Node() << Node();
//    //mNodes.resize(mNodes.size()+2);

//    {
//        // left child
//        unsigned int childId = mNodes[nodeId].firstChildId;
//        KdNode& child = mNodes[childId];
//        if (child.triangleIndices <= targetCellSize || level>=targetMaxDepth)
//        {
//            child.leaf = 1;
//        }
//        else
//        {
//            child.leaf = 0;
//            createTree(childId, child.triangleIndices, level+1, targetCellSize, targetMaxDepth);
//        }
//    }

//    {
//        // right child
//        unsigned int childId = mNodes[nodeId].firstChildId+1;
//        KdNode& child = mNodes[childId];
//        if (child.triangleIndices <= targetCellSize || level>=targetMaxDepth)
//        {
//            child.leaf = 1;
//        }
//        else
//        {
//            child.leaf = 0;
//            createTree(childId, child.triangleIndices, level+1, targetCellSize, targetMaxDepth);
//        }
//    }
//}

//} //namespace Super4PCS

#endif // TRIANGLEKDTREE_H

