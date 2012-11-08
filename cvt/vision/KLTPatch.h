/*
            CVT - Computer Vision Tools Library

     Copyright (c) 2012, Philipp Heise, Sebastian Klose

    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
    PARTICULAR PURPOSE.
 */
#ifndef CVT_KLT_PATCH_H
#define CVT_KLT_PATCH_H

#include <Eigen/Dense>
#include <cvt/vision/ImagePyramid.h>
#include <cvt/gfx/IMapScoped.h>
#include <cvt/util/EigenBridge.h>
#include <cvt/util/CVTAssert.h>
#include <Eigen/StdVector>

namespace cvt
{
    template <size_t pSize, class PoseType>
    class KLTPatch
    {
        public:
            typedef Eigen::Matrix<float, PoseType::NPARAMS, PoseType::NPARAMS> HessType;
            typedef Eigen::Matrix<float, PoseType::NPARAMS, 1>                 JacType;
            static const size_t PatchSize = pSize;

            KLTPatch( size_t octaves = 1 );

            /**
             *	\brief update the internal patch data
             *	\param ptr		ptr to the image
             *	\param stride	stride of the image
             *	\param pos		position of the feature in the image
             * */
            bool update( IMapScoped<const float> &iMap,
                         IMapScoped<const float> &gxMap,
                         IMapScoped<const float> &gyMap,
                         const Vector2f &pos,
                         size_t w, size_t h, size_t octave = 0 );

            bool update( const ImagePyramid & pyrImg,
                         const ImagePyramid & pyrGx,
                         const ImagePyramid & pyrGy,
                         const Vector2f &pos );

            void currentCenter( Vector2f& center )  const;

            PoseType&	pose()	{ return _pose; }
            void		initPose( const Vector2f& pos );

            const float*   pixels( size_t octave = 0 )       const { return _patchDataForScale[ octave ].patch; }
            const float*   transformed( size_t octave = 0 )  const { return _patchDataForScale[ octave ].transformed; }

            const HessType&  inverseHessian( size_t octave = 0 ) const { return _patchDataForScale[ octave ].inverseHessian; }
            const JacType*   jacobians( size_t octave = 0 )      const { return _patchDataForScale[ octave ].jac; }

            static size_t size() { return pSize; }

            bool align( const float *current, size_t currStride,
                        size_t width, size_t height, size_t maxIters = 2 );

            /* track patch through pyramid */
            bool align( const ImagePyramid& pyramid, size_t maxIters = 2 );


            /**
             *	\brief	number of scales that are stored with this patch
             * */
            size_t numScales() const { return _patchDataForScale.size(); }

            static void extractPatches( std::vector<KLTPatch<pSize, PoseType>* > & patches,
                                        const std::vector<Vector2f> & positions,
                                        const Image & img,
                                        const Image & gradX,
                                        const Image & gradY );

            /**
             * \brief extract patches a multiscale fashion from a image pyramid
             * */
            static void extractPatches( std::vector<KLTPatch<pSize, PoseType>* > & patches,
                                        const std::vector<Vector2f> & positions,
                                        const ImagePyramid & pyramid,
                                        const ImagePyramid & gradX,
                                        const ImagePyramid & gradY );            

            static const Vector2f* patchPoints()    { return &PatchPoints[ 0 ]; }
            static size_t          numPatchPoints() { return PatchPoints.size(); }

            void toImage( Image& img, size_t octave = 0 ) const;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            /* this stores the transform from the
             * Template space to the image space*/
            PoseType	_pose;

            struct PatchData
            {
                /* the pixel original information */
                float		patch[ pSize * pSize ];

                /* the transformed information */
                float		transformed[ pSize * pSize ];

                HessType	inverseHessian;
                JacType		jac[ pSize * pSize ];

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

            typedef std::vector<PatchData, Eigen::aligned_allocator<PatchData> > PatchDataVec;
            PatchDataVec	_patchDataForScale;

            KLTPatch( const KLTPatch& );
            KLTPatch& operator= (const KLTPatch& );

            static std::vector<Vector2f> PatchPoints;
            static std::vector<Vector2f> initPatchPoints()
            {
                std::vector<Vector2f> points;
                points.reserve( pSize * pSize );

                int half = pSize >> 1;
                Vector2f p( -half, -half );
                for( size_t rows = 0; rows < pSize; rows++ ){
                    p.x = -half;
                    for( size_t cols = 0; cols < pSize; cols++ ){
                        points.push_back( p );
                        p.x += 1.0f;
                    }
                    p.y += 1.0f;
                }
                return points;
            }

            bool patchIsInImage( const Matrix3f& pose, size_t w, size_t h ) const;
            float buildSystem( JacType& jacSum,
                               const Matrix3f& pose,
                               const float *iPtr, size_t iStride,
                               size_t width, size_t height,
                               size_t octave = 0 );
    };

    template <size_t pSize, class PoseType>
    std::vector<Vector2f> KLTPatch<pSize, PoseType>::PatchPoints( KLTPatch<pSize, PoseType>::initPatchPoints() );


    template <size_t pSize, class PoseType>
    inline KLTPatch<pSize, PoseType>::KLTPatch( size_t octaves )
    {
        _patchDataForScale.resize( octaves );
    }

    template <size_t pSize, class PoseType>
    inline bool KLTPatch<pSize, PoseType>::update( IMapScoped<const float>& iMap,
                                                   IMapScoped<const float>& gxMap,
                                                   IMapScoped<const float>& gyMap,
                                                   const Vector2f& pos, size_t w, size_t h, size_t octave )
    {
        const float pHalf = ( pSize >> 1 );

        if( pos.x < pHalf || ( pos.x > w - pHalf - 1 ) ||
            pos.y < pHalf || ( pos.y > h - pHalf - 1 ) )
            return false;

        size_t stride = iMap.stride() / sizeof( float );
        size_t offset = ( int )( pos.y - pHalf ) * stride + ( int )( pos.x - pHalf );

        // images have same type and type -> same stride, get pointer to first pixel
        const float* iptr = iMap.ptr() + offset;
        const float* gxptr = gxMap.ptr() + offset;
        const float* gyptr = gyMap.ptr() + offset;

        size_t numLines = pSize;

        JacType* J = _patchDataForScale[ octave ].jac;
        HessType& invH = _patchDataForScale[ octave ].inverseHessian;
        float* p = _patchDataForScale[ octave ].patch;
        float* t = _patchDataForScale[ octave ].transformed;

        Eigen::Matrix<float, 2, 1> g;
        HessType hess( HessType::Zero() );
        typename PoseType::ScreenJacType sj;

        Eigen::Vector2f point( -pHalf, -pHalf );
        while( numLines-- )
        {
            point[ 0 ] = -pHalf;
            for( size_t i = 0; i < pSize; i++ ){
                *p = *t = iptr[ i ];
                g[ 0 ] = gxptr[ i ];
                g[ 1 ] = gyptr[ i ];

                _pose.screenJacobian( sj, point );
                *J =  sj.transpose() * g;

                hess.noalias() += ( *J ) * J->transpose();
                J++;
                p++;
                t++;

                point[ 0 ] += 1.0f;
            }
            iptr  += stride;
            gxptr += stride;
            gyptr += stride;

            point[ 1 ] += 1.0f;
        }

        // initialize the _pose if at upper most octave
        if( octave == 0 )
            initPose( pos );

        float det = hess.determinant();
        if( Math::abs( det ) > 1e-5 ){
            invH = hess.inverse();
            return true;
        }

        return false;
    }

    template <size_t pSize, class PoseType>
    inline bool KLTPatch<pSize, PoseType>::update( const ImagePyramid & pyrImg,
                                                   const ImagePyramid & pyrGx,
                                                   const ImagePyramid & pyrGy,
                                                   const Vector2f &pos )
    {
        Vector2f scalePos = pos;
        for( size_t i = 0; i < pyrImg.octaves(); i++ ){
            IMapScoped<const float> iMap( pyrImg[ i ] );
            IMapScoped<const float> gxMap( pyrGx[ i ] );
            IMapScoped<const float> gyMap( pyrGy[ i ] );

            if( !this->update( iMap, gxMap, gyMap, scalePos, pyrImg[ i ].width(), pyrImg[ i ].height(), i ) )
                return false;
            scalePos *= pyrImg.scaleFactor();
        }
        return true;
    }

    template <size_t pSize, class PoseType>
    inline void KLTPatch<pSize, PoseType>::extractPatches( std::vector<KLTPatch<pSize, PoseType>* > & patches,
                                                           const std::vector<Vector2f> & positions,
                                                           const Image & img,
                                                           const Image& gradX,
                                                           const Image& gradY )
    {
        IMapScoped<const float> iMap( img );
        IMapScoped<const float> gxMap( gradX );
        IMapScoped<const float> gyMap( gradY );

        size_t w = img.width();
        size_t h = img.height();
        size_t phalf = pSize >> 1;

        int x, y;

        KLTPatch<pSize, PoseType>* patch = 0;

        for( size_t i = 0; i < positions.size(); i++ ){
            x = positions[ i ].x;
            y = positions[ i ].y;

            if( x < ( int )phalf || ( x + phalf - 1 ) > w ||
                y < ( int )phalf || ( y + phalf - 2 ) > h )
                continue;

            if( patch == 0 )
                patch = new KLTPatch<pSize, PoseType>();

            if( patch->update( iMap, gxMap, gyMap, positions[ i ] ) ){
                patches.push_back( patch );
                patch = 0;
            }
        }

        if( patch )
            delete patch;
    }

    template <size_t pSize, class PoseType>
    inline void KLTPatch<pSize, PoseType>::extractPatches( std::vector<KLTPatch<pSize, PoseType>* > & patches,
                                                           const std::vector<Vector2f> & positions,
                                                           const ImagePyramid & pyr,
                                                           const ImagePyramid & gradX,
                                                           const ImagePyramid & gradY )
    {
        int x, y;

        KLTPatch<pSize, PoseType>* patch = 0;
        std::vector<float> scales;

        std::vector<IMapScoped<const float>*> iMaps;
        std::vector<IMapScoped<const float>*> gxMaps;
        std::vector<IMapScoped<const float>*> gyMaps;

        scales.resize( pyr.octaves() );

        float scale = 1.0f;
        for( size_t i = 0; i < pyr.octaves(); i++ ){
            iMaps.push_back( new IMapScoped<const float>( pyr[ i ] ) );
            gxMaps.push_back( new IMapScoped<const float>( gradX[ i ] ) );
            gyMaps.push_back( new IMapScoped<const float>( gradY[ i ] ) );
            scales[ i ] = scale;
            scale *= pyr.scaleFactor();
        }

        Vector2f octavePos;
        size_t phalf = pSize >> 1;
        for( size_t i = 0; i < positions.size(); i++ ){

            float fx = positions[ i ].x;
            float fy = positions[ i ].y;

            bool isGood = true;
            if( patch == 0 )
                patch = new KLTPatch<pSize, PoseType>( pyr.octaves() );
            for( int o = pyr.octaves()-1; o >= 0; o-- ){
                x = scales[ o ] * fx;
                y = scales[ o ] * fy;

                size_t w = pyr[ o ].width();
                size_t h = pyr[ o ].height();

                octavePos = positions[ i ] * scales[ o ];

                if( x < ( int )phalf + 1 || ( x + phalf + 1 ) >= w ||
                    y < ( int )phalf + 1 || ( y + phalf + 1 ) >= h ){
                    isGood = false;
                    break;
                }

                // update octave o of patch
                if( !patch->update( *iMaps[ o ], *gxMaps[ o ], *gyMaps[ o ], octavePos, w, h, o ) ){
                    isGood = false;
                    break;
                }
            }

            if( isGood ){
                patches.push_back( patch );
                patch = 0;
            }
        }

        if( patch )
            delete patch;

        // unmap
        for( size_t i = 0; i < iMaps.size(); i++ ){
            delete iMaps[ i ];
            delete gxMaps[ i ];
            delete gyMaps[ i ];
        }
    }

    template <size_t pSize, class PoseType>
    inline bool KLTPatch<pSize, PoseType>::align( const float *current, size_t currStride,
                                                  size_t width, size_t height,
                                                  size_t maxIters )
    {
        JacType jSum;
        typename PoseType::ParameterVectorType delta;

        const PatchData& patchData = _patchDataForScale[ 0 ];

        float diffSum = 0.0f;
        const float maxDiff = 0.7 * Math::sqr( pSize * 255.0f );

        size_t iter = 0;
        while( iter < maxIters ){
            jSum.setZero();
            diffSum = 0.0f;

            //pose matrix
            Matrix3f pose;
            EigenBridge::toCVT( pose, _pose.transformation() );

            // first test if all points transform into the image
            if( !patchIsInImage( pose, width, height ) ){
                return false;
            }

            diffSum = buildSystem( jSum, pose, current, currStride, width, height );
            if( diffSum > maxDiff )
                return false;

            // solve for the delta:
            delta = patchData.inverseHessian * jSum;
            _pose.applyInverse( -delta );

            iter++;
        }
        return true;
    }

    /* track patch through pyramid */
    template <size_t pSize, class PoseType>
    inline bool KLTPatch<pSize, PoseType>::align( const ImagePyramid& pyramid, size_t maxIters )
    {
        CVT_ASSERT( pyramid[ 0 ].format() == IFormat::GRAY_FLOAT, "Format must be GRAY_FLOAT!" );

        JacType jSum;
        typename PoseType::ParameterVectorType delta;

        size_t nOctaves = pyramid.octaves();
        float scale = Math::pow( pyramid.scaleFactor(), nOctaves-1 );
        float invScale = 1.0f / pyramid.scaleFactor();

        // get the pose of the patch
        Matrix3f poseMat;
        EigenBridge::toCVT( poseMat, _pose.transformation() );
        poseMat[ 0 ][ 2 ] *= scale;
        poseMat[ 1 ][ 2 ] *= scale;

        PoseType tmpPose;
        tmpPose.set( poseMat );

        const float maxDiff = Math::sqr( pSize * 255.0f );
        float diffSum = maxDiff;
        for( int oc = pyramid.octaves() - 1; oc >= 0; --oc ){
            IMapScoped<const float> map( pyramid[ oc ] );
            const PatchData& patchData = _patchDataForScale[ oc ];
            size_t w = pyramid[ oc ].width();
            size_t h = pyramid[ oc ].height();
            size_t iter = 0;
            diffSum = Math::sqr( pSize * 255 );
            while( iter < maxIters ){
                //pose matrix
                EigenBridge::toCVT( poseMat, tmpPose.transformation() );

                // first test if all points of the patch transform into the image
                if( !patchIsInImage( poseMat, w, h ) ){
                    return false;
                }

                jSum.setZero();
                diffSum = buildSystem( jSum, poseMat,
                                       map.ptr(), map.stride(),
                                       w, h,
                                       oc );                

                if( diffSum >= maxDiff ){
                    return false;
                }

                // solve for the delta:
                delta = patchData.inverseHessian * jSum;

                if( delta.norm() < 1e-6 )
                    break;

                tmpPose.applyInverse( -delta );
                iter++;
            }

            if( oc != 0 ){
                // we need to scale up
                EigenBridge::toCVT( poseMat, tmpPose.transformation() );
                poseMat[ 0 ][ 2 ] *= invScale;
                poseMat[ 1 ][ 2 ] *= invScale;
                tmpPose.set( poseMat );
            }
        }

        // set the final patch pose accordingly
        _pose.transformation() = tmpPose.transformation();
        return true;
    }


    template <size_t pSize, class PoseType>
    inline float KLTPatch<pSize, PoseType>::buildSystem( JacType& jacSum,
                                                         const Matrix3f& pose,
                                                         const float* imgPtr, size_t iStride,
                                                         size_t width, size_t height,
                                                         size_t octave )
    {
        // the warped current ones
        PatchData& data = _patchDataForScale[ octave ];
        float* warped = data.transformed;
        // the original template pixels
        const float* temp = data.patch;
        const JacType* J = data.jac;

        float diffSum = 0.0f;

        // check for nans in matrix
        for( size_t i = 0; i < 3; i++ )
            for( size_t k = 0; k < 3; k++ )
                if( Math::isNaN( pose[ i ][ k ] ) )
                    return Math::MAXF;

        SIMD* simd = SIMD::instance();
        // transform the points:
        std::vector<Vector2f> warpedPts( numPatchPoints() );
        simd->transformPoints( &warpedPts[ 0 ], pose, patchPoints(), warpedPts.size() );
        simd->warpBilinear1f( warped, &warpedPts[ 0 ].x, imgPtr, iStride, width, height, -1.0f, warpedPts.size() );

        // compute the residuals
        float residuals[ pSize * pSize ];
        simd->Sub( residuals, warped, temp, warpedPts.size() );

        size_t num = warpedPts.size();
        const float* r = residuals;
        while( num-- ){
            diffSum += Math::sqr( *r );
            jacSum += ( *J *  *r );
            J++;
            r++;
        }
        return diffSum;
    }


    template <size_t pSize, class PoseType>
    inline void KLTPatch<pSize, PoseType>::currentCenter( Vector2f& center ) const
    {
        const Eigen::Matrix3f& tmp = _pose.transformation();
        center.x = tmp( 0, 2 );
        center.y = tmp( 1, 2 );
    }

    template <size_t pSize, class PoseType>
    inline void KLTPatch<pSize, PoseType>::initPose( const Vector2f& pos )
    {
        Matrix3f m;
        m.setIdentity();
        m[ 0 ][ 2 ] = pos.x;
        m[ 1 ][ 2 ] = pos.y;
        _pose.set( m );
    }

    template <size_t pSize, class PoseType>
    inline void KLTPatch<pSize, PoseType>::toImage( Image& img, size_t octave ) const
    {
        img.reallocate( pSize, pSize, IFormat::GRAY_FLOAT );
        IMapScoped<uint8_t> map( img );
        int r = pSize;
        const uint8_t* vals = ( const uint8_t* )pixels( octave );
        SIMD* simd = SIMD::instance();
        size_t stride = pSize * sizeof( float );
        while( r-- ){
            simd->Memcpy( map.ptr(), vals, stride );
            vals += stride;
            map++;
        }
    }

    template <size_t pSize, class PoseType>
    bool KLTPatch<pSize, PoseType>::patchIsInImage( const Matrix3f& pose, size_t w, size_t h ) const
    {
        static const float half = pSize >> 1;
        static const Vector2f a( -half, -half );
        static const Vector2f b(  half, -half );
        static const Vector2f c(  half,  half );
        static const Vector2f d( -half,  half );

        Vector2f pWarped;

        pWarped = pose * a;
        if( pWarped.x < 0.0f || pWarped.x >= w ||
            pWarped.y < 0.0f || pWarped.y >= h )
            return false;

        pWarped = pose * b;
        if( pWarped.x < 0.0f || pWarped.x >= w ||
            pWarped.y < 0.0f || pWarped.y >= h )
            return false;

        pWarped = pose * c;
        if( pWarped.x < 0.0f || pWarped.x >= w ||
            pWarped.y < 0.0f || pWarped.y >= h )
            return false;

        pWarped = pose * d;
        if( pWarped.x < 0.0f || pWarped.x >= w ||
            pWarped.y < 0.0f || pWarped.y >= h )
            return false;
        return true;
    }
}

#endif
