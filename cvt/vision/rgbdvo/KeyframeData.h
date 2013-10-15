#ifndef CVT_KEYFRAMEDATA_H
#define CVT_KEYFRAMEDATA_H

#include <Eigen/Core>
#include <cvt/gfx/Image.h>
#include <cvt/gfx/IMapScoped.h>

namespace cvt {

    /**
     * \class AlignmentData for reference (template) information
     */
    template <class Warp>
    class AlignmentData {
        public:
            typedef Warp                            WarpType;
            typedef typename Warp::JacobianType     JacobianType;
            typedef typename Warp::ScreenJacType    ScreenJacobianType;
            typedef typename Warp::HessianType      HessianType;
            typedef Eigen::Matrix<float, 1, 2>      GradientType;
            typedef std::vector<ScreenJacobianType, Eigen::aligned_allocator<ScreenJacobianType> > ScreenJacVec;
            typedef std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> > JacobianVec;

            Image                       gray;
            Image                       gradX;
            Image                       gradY;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            void reserve( size_t size )
            {
                _points3d.reserve( size );
                _pixelValues.reserve( size );
                _jacobians.reserve( size );
                _screenJacobians.reserve( size );
            }

            void clear()
            {
                _points3d.clear();
                _pixelValues.clear();
                _jacobians.clear();
                _screenJacobians.clear();
            }

            void add( const Vector3f& point,
                      const ScreenJacobianType& jac,
                      const GradientType& iGrad,
                      float val )
            {
                _points3d.push_back( point );
                _screenJacobians.push_back( jac );
                _jacobians.push_back( JacobianType() );
                Warp::computeJacobian( _jacobians.back(), jac, iGrad, val );
                _pixelValues.push_back( val );
            }

            size_t size() const { return _points3d.size(); }

            const Matrix3f& intrinsics() const { return _intrinsics; }
            void setIntrinsics( const Matrix3f& intr ){ _intrinsics = intr; }

            const std::vector<Vector3f>&     points()    const { return _points3d; }
            const std::vector<float>&        pixels()    const { return _pixelValues; }

            // these are the offline jacobians
            const JacobianVec& jacobians() const { return _jacobians; }
            const ScreenJacVec& screenJacobians() const { return _screenJacobians; }

            void selectInformation( size_t n )
            {
                if( size() <= n )
                    return;

                InformationSelection<JacobianType> selector( n );
                const std::set<size_t>& ids = selector.selectInformation( &_jacobians[ 0 ], _jacobians.size() );

                // now rearrange the data according to the ids:
                std::set<size_t>::const_iterator it = ids.begin();
                const std::set<size_t>::const_iterator end = ids.end();

                size_t saveIdx = 0;
                while( it != end ){
                    _jacobians[ saveIdx ] = _jacobians[ *it ];
                    _points3d[ saveIdx ] = _points3d[ *it ];
                    _pixelValues[ saveIdx ] = _pixelValues[ *it ];
                    _screenJacobians[ saveIdx ] = _screenJacobians[ *it ];

                    ++saveIdx;
                    ++it;
                }

                // remove the rest
                _jacobians.erase( _jacobians.begin() + n, _jacobians.end() );
                _points3d.erase( _points3d.begin() + n, _points3d.end() );
                _pixelValues.erase( _pixelValues.begin() + n, _pixelValues.end() );
                _screenJacobians.erase( _screenJacobians.begin() + n, _screenJacobians.end() );
            }

            static void interpolateGradients( std::vector<float>& result, const Image& gradImg, const std::vector<Vector2f>& positions, const SIMD* simd )
            {
                IMapScoped<const float> map( gradImg );
                simd->warpBilinear1f( &result[ 0 ], &positions[ 0 ].x, map.ptr(), map.stride(), gradImg.width(), gradImg.height(), -20.0f, positions.size() );
            }

        protected:
            std::vector<Vector3f>       _points3d;
            std::vector<float>          _pixelValues;
            JacobianVec                 _jacobians;
            ScreenJacVec                _screenJacobians;
            Matrix3f                    _intrinsics;
    };

}

#endif // CVT_KEYFRAMEDATA_H
