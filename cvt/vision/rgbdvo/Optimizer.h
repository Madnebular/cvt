/*
            CVT - Computer Vision Tools Library

     Copyright (c) 2012, Philipp Heise, Sebastian Klose

    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
    PARTICULAR PURPOSE.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <cvt/vision/rgbdvo/RGBDKeyframe.h>
#include <cvt/vision/rgbdvo/SystemBuilder.h>
#include <Eigen/LU>

#include <Eigen/LU>

namespace cvt {

    class HistMedianSelect {
        public:
            HistMedianSelect( float min, float max, float resolution ) :
                _min( min ),
                _max( max ),
                _range( _max - _min ),
                _resolution( resolution )
            {
                size_t nBins = _range / resolution;
                _hist.resize( nBins, 0 );
            }

            void add( float value )
            {
                value = Math::clamp( value, _min, _max );
                float fidx =  value / ( _range ) * ( float ) ( _hist.size() - 1 ) + 0.5f;
                int idx = ( int ) fidx;

                _hist[ idx ] += 1;
            }

            // approximate the nth value
            float approximateNth( size_t nth )
            {
                size_t bin = 0;
                size_t num = _hist[ bin++ ];

                while( num < nth ){
                    num += _hist[ bin++ ];
                }
                bin--;
                size_t nPrev = num - _hist[ bin ];

                if( bin )
                    bin--;

                // previous is smaller:
                float frac = ( nth - nPrev ) / ( float )( num - nPrev );

                return ( bin + frac ) * _resolution;
            }

            void clearHistogram()
            {
                for( size_t i = 0; i < _hist.size(); i++ ){
                    _hist[ i ] = 0;
                }
            }

        private:
            float               _min;
            float               _max;
            float               _range;
            float               _resolution;
            std::vector<size_t> _hist;
    };


    template <class WarpFunc, class Weighter>
    class Optimizer
    {
        public:
            typedef typename RGBDKeyframe<WarpFunc>::Result   ResultType;

            Optimizer();
            virtual ~Optimizer(){}

            void setMaxIterations( size_t iter ) { _maxIter = iter; }
            void setMinUpdate( float v )         { _minUpdate = v; }
            void setRobustThreshold( float v )   { _robustThreshold = v; }

            virtual void optimize( ResultType& result,
                                   const Matrix4f& posePrediction,
                                   RGBDKeyframe<WarpFunc>& reference,
                                   const ImagePyramid& grayPyramid,
                                   const Image& depthImage ) const;


        protected:
            typedef typename WarpFunc::JacobianType     JacobianType;
            typedef typename WarpFunc::HessianType      HessianType;
            typedef typename WarpFunc::DeltaVectorType  DeltaType;
            typedef typename RGBDKeyframe<WarpFunc>::AlignDataType AlignDataType;
            size_t  _maxIter;
            float   _minUpdate;
            float   _robustThreshold;

            float computeMedian( const float* residuals, const std::vector<size_t>& indices ) const
            {
                HistMedianSelect medianSelector( 0.0f, 1.0f, 0.01f );

                std::vector<size_t>::const_iterator it = indices.begin();
                const std::vector<size_t>::const_iterator end = indices.end();
                while( it != end ){
                    medianSelector.add( Math::abs( residuals[ *it ] ) );
                    ++it;
                }

                return medianSelector.approximateNth( indices.size() >> 1 );
            }

            void validIndices( std::vector<size_t>& indices, const float* vals, size_t num, float minVal ) const
            {
                indices.clear();
                indices.reserve( num );

                for( size_t i = 0; i < num; i++ ){
                    if( vals[ i ] > minVal ){
                        indices.push_back( i );
                    }
                }
            }

            bool testIndices( const std::vector<size_t>& foundValid, const std::vector<Vector2f>& pts, size_t width, size_t height ) const
            {
                std::vector<size_t>::const_iterator it = foundValid.begin();
                const std::vector<size_t>::const_iterator itEnd = foundValid.end();

                while( it != itEnd ){

                    if( *it >= pts.size() ){
                        std::cerr << "Index out of bounds: pts[ " << *it << " ], " << " pts size:" <<  pts.size() << std::endl;
                        return false;
                    }

                    const Vector2f& p = pts[ *it ];
                    if( p.x < 0 || p.x >= width ||
                        p.y < 0 || p.y >= height ){
                        std::cerr << "Point out of image: " << p << " [w,h]: [" << width << ", " << height << "]" << std::endl;
                        return false;
                    }

                    ++it;
                }

                return true;
            }

    };

    template <class WarpFunc, class LossFunc>
    inline Optimizer<WarpFunc, LossFunc>::Optimizer() :
        _maxIter( 10 ),
        _minUpdate( 1e-6 ),
        _robustThreshold( 0.1f )
    {
    }

    template <class WarpFunc, class LossFunc>
    inline void Optimizer<WarpFunc, LossFunc>::optimize( ResultType& result,
                                                         const Matrix4f& posePrediction,
                                                         RGBDKeyframe<WarpFunc> &reference,
                                                         const ImagePyramid& grayPyramid,
                                                         const Image& /*depthImage*/ ) const
    {
        SIMD* simd = SIMD::instance();
        Matrix4f tmp4;
        tmp4 = posePrediction.inverse() * reference.pose();

        result.warp.setPose( tmp4 );
        result.costs = 0.0f;

        result.iterationsOnOctave.resize( grayPyramid.octaves(), 0 );
        result.numPixels = 0;
        result.pixelPercentage = 0.0f;

        LossFunc weighter( _robustThreshold );
        SystemBuilder<LossFunc> builder( weighter );

        Matrix4f projMat;
        std::vector<Vector2f> warpedPts;
        std::vector<float> interpolatedPixels;
        std::vector<float> interpolatedGx;
        std::vector<float> interpolatedGy;
        std::vector<float> residuals;
        std::vector<size_t> indices;

        // TODO: compose this into one multichannel image
        ImagePyramid gradX( grayPyramid.octaves(), grayPyramid.scaleFactor() );
        ImagePyramid gradY( grayPyramid.octaves(), grayPyramid.scaleFactor() );

        grayPyramid.convolve( gradX, reference.kernelDx() );
        grayPyramid.convolve( gradY, reference.kernelDy());

        // sum of jacobians * delta
        JacobianType deltaSum;
        HessianType  hessian;

        std::vector<JacobianType> esmJacobian;

        for( int o = grayPyramid.octaves() - 1; o >= 0; o-- ){
            ResultType scaleResult;
            scaleResult = result;

            const size_t width = grayPyramid[ o ].width();
            const size_t height = grayPyramid[ o ].height();

            const AlignDataType& data = reference.dataForScale( o );

            const size_t num = data.points3d.size();
            Matrix4f K4( data.intrinsics );
            const Vector3f* p3dPtr = &data.points3d[ 0 ];
            const Vector2f* refGrads = &data.gradients[ 0 ];

            const float* referencePixVals = &data.pixelValues[ 0 ];
            //const JacobianType* referenceJ = &data.jacobians[ 0 ];

            warpedPts.resize( num );
            interpolatedPixels.resize( num );
            interpolatedGx.resize( num );
            interpolatedGy.resize( num );
            residuals.resize( num );
            esmJacobian.resize( num );

            IMapScoped<const float> grayMap( grayPyramid[ o ] );
            IMapScoped<const float> gxMap( gradX[ o ] );
            IMapScoped<const float> gyMap( gradY[ o ] );

            scaleResult.iterationsOnOctave[ o ] = 0;
            scaleResult.numPixels = 0;
            scaleResult.pixelPercentage = 0.0f;

            while( scaleResult.iterationsOnOctave[ o ] < _maxIter ){
                // build the updated projection Matrix
                projMat = K4 * scaleResult.warp.poseMatrix();

                // project the points:
                simd->projectPoints( &warpedPts[ 0 ], projMat, p3dPtr, num );

                // interpolate the pixel values
                simd->warpBilinear1f( &interpolatedPixels[ 0 ], &warpedPts[ 0 ].x, grayMap.ptr(), grayMap.stride(), width, height, -10.0f, num );
                scaleResult.warp.computeResiduals( &residuals[ 0 ], referencePixVals, &interpolatedPixels[ 0 ], num );

                // compute the valid indices: (those that are within the image)
                validIndices( indices, &interpolatedPixels[ 0 ], num, -0.1f );

                float median = computeMedian( &residuals[ 0 ], indices );

                /* this is an estimate for the standard deviation:
                 * TODO: check if this is the same as described in the paper
                 */
                weighter.setSigma( 1.4f * median );


                // interpolate the warped gradient for ESM
                simd->warpBilinear1f( &interpolatedGx[ 0 ], &warpedPts[ 0 ].x, gxMap.ptr(), gxMap.stride(), width, height, -20.0f, num );
                simd->warpBilinear1f( &interpolatedGy[ 0 ], &warpedPts[ 0 ].x, gyMap.ptr(), gyMap.stride(), width, height, -20.0f, num );

                Eigen::Vector2f gVec;
                std::vector<size_t>::const_iterator idxIter = indices.begin();
                const std::vector<size_t>::const_iterator idxIterEnd = indices.end();

                while( idxIter != idxIterEnd ){
                    gVec[ 0 ] = refGrads[ *idxIter ].x;
                    gVec[ 1 ] = refGrads[ *idxIter ].y;

                    if( interpolatedGx[ *idxIter ] != -20.0f ){
                        gVec[ 0 ] += interpolatedGx[ *idxIter ];
                        gVec[ 0 ] *= 0.5f;
                    }
                    if( interpolatedGy[ *idxIter ] != -20.0f ){
                        gVec[ 1 ] += interpolatedGy[ *idxIter ];
                        gVec[ 1 ] *= 0.5f;
                    }
                    esmJacobian[ *idxIter ] = gVec.transpose() * data.screenJacobians[ *idxIter ];
                    ++idxIter;
                }

                /* a hack: the builder does not touch the hessian if its a non robust lossfunc!*/
                hessian = data.hessian;
                scaleResult.numPixels = builder.build( hessian, deltaSum,
                                                       //referenceJ,
                                                       &esmJacobian[ 0 ],
                                                       &residuals[ 0 ],
                                                       indices,
                                                       scaleResult.costs );
                if( !scaleResult.numPixels ||
                    scaleResult.costs / scaleResult.numPixels < 0.005f ){
                    break;
                }

                // evaluate the delta parameters
                Eigen::FullPivLU<HessianType> lu( hessian );

                if( !lu.isInvertible() ){
                    std::cout << "Hessian is not invertible\n" << hessian << std::endl;
                    getchar();

                    break;
                }
                DeltaType deltaP = -lu.inverse() * deltaSum.transpose();
                scaleResult.warp.updateParameters( deltaP );

                /*
                std::cout << "Scale:\t" << o << "\tCosts:\t" << scaleResult.costs << "\tDelta:" <<
                             deltaP[ 0 ] << ", " <<
                             deltaP[ 1 ] << ", " <<
                             deltaP[ 2 ] << ", " <<
                             deltaP[ 3 ] << ", " <<
                             deltaP[ 4 ] << ", " <<
                             deltaP[ 5 ] <<  std::endl;
                std::cout << "NumPixel: " << scaleResult.numPixels << std::endl;
                std::cout << "Pose:\n" << scaleResult.warp.poseMatrix()  << "\n" << std::endl;*/

                scaleResult.iterationsOnOctave[ o ]++;

                if( deltaP.norm() < _minUpdate )
                    break;
            }

            if( scaleResult.numPixels )
                scaleResult.pixelPercentage = ( float )scaleResult.numPixels / ( float )num;

            // TODO: ensure the result on this scale is good enough (pixel percentage & error )
            result = scaleResult;
        }

        tmp4 = result.warp.poseMatrix();
        tmp4 = reference.pose() * tmp4.inverse();
        result.warp.setPose( tmp4 );
        std::cout << "Current Pose: \n" << tmp4 << std::endl;
    }

}

#endif // OPTIMIZER_H
