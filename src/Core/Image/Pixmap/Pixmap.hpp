#ifndef RADIUMENGINE_PIXMAP_HPP
#define RADIUMENGINE_PIXMAP_HPP

#include <Core/RaCore.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Containers/VectorArray.hpp>
#include <Core/Image/stb_image.h>
#include <Core/Image/stb_image_write.h>

namespace Ra
{
namespace Core
{

    typedef Eigen::Matrix<uint,Eigen::Dynamic, Eigen::Dynamic> Pixmap;
    typedef Eigen::Transform<Scalar, 2, Eigen::Affine>  Transform2D;


    inline Pixmap loadPixFromStb( const uchar* data, uint w, uint h, int ncomp )
    {
        Pixmap result(h,w);
        for (uint i = 0; i < h; ++i)
        {
            for (uint j = 0; j < w; ++j)
            {
                const uchar* pix = data + (ncomp * ( j + i * w));

                uint nonzero = 0;
                // follow stb image formats channels and ignore alpha
                // 1 : greyscale
                // 2 : grey + alpha
                // 3 : rgb
                // 4 : rgba
                if (ncomp <3 && *pix != 0)
                {
                    nonzero = 1;
                }
                else if ( pix[0] + pix[1] + pix[2] > 0)
                {
                    nonzero = 1;
                }

                result(i,j) = nonzero;
            }
        }
        return result;
    }


    inline ObbD<2> pca(const Pixmap& pix)
    {
        Vector2Array pts;
        for (uint i =0; i < pix.rows(); ++i )
        {
            for (uint j =0; j < pix.cols(); ++j )
            {
                if (pix(i,j) !=0)
                {
                    pts.push_back(Vector2{i,j});
                }
            }
        }

        Vector2 avg = pts.getMap().rowwise().mean();

        Vector2Array ptsAvg(pts.size());

        ptsAvg.getMap() = pts.getMap().colwise() - avg;

        MatrixN vCov = (1.f / (pts.size() -1)) * (ptsAvg.getMap() * ptsAvg.getMap().transpose());

        Eigen::SelfAdjointEigenSolver<Matrix2> solver(vCov);

        Transform2D pca;
        pca.translation() = avg; //(need axis inversion maybe )
        pca.linear() = solver.eigenvectors();

        Transform2D pcaInv = pca.inverse();

        Vector2Array alignedPts;
        for ( const auto& v : pts)
        {
            alignedPts.push_back(pcaInv * v);
        }

        Eigen::AlignedBox2f alignedBox( alignedPts.getMap().rowwise().minCoeff(),
                                        alignedPts.getMap().rowwise().maxCoeff()) ;
        ObbD<2> obb ( alignedBox, pca);
        return obb;

    }


    inline void doit()
    {

        int w,h,n;
        uchar* data = stbi_load("test.png", &w, &h,&n, 0);

        Pixmap p = loadPixFromStb( data,w, h,n);

        ObbD<2> obb = pca(p);


        for (uint c =0; c < 4; ++c)
        {
            auto corner = obb.worldCorner(c);
            const uint i = corner.x();
            const uint j = corner.y();

            uchar* pix = data + (n * ((j + i * w)));
            pix[0] = 255;
            pix[1] = 0;
            pix[2] = 0;
        }

        stbi_write_png("test_x.png", w, h ,n, data, w * n * sizeof(uchar));
        stbi_image_free(data);
    }



}
}


#endif //RADIUMENGINE_PIXMAP_HPP
