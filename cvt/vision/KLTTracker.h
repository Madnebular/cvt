/*
			CVT - Computer Vision Tools Library

 	 Copyright (c) 2012, Philipp Heise, Sebastian Klose

 	THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 	PARTICULAR PURPOSE.
 */
#ifndef CVT_KLT_TRACKER_H
#define CVT_KLT_TRACKER_H

#include <cvt/math/Vector.h>
#include <cvt/gfx/Image.h>
#include <vector>

#include <cvt/vision/KLTPatch.h>
#include <cvt/math/Translation2D.h>
#include <cvt/util/EigenBridge.h>

namespace cvt
{
	template <class PoseType, size_t pSize=16>
	class KLTTracker
	{
		public:
			typedef KLTPatch<pSize, PoseType>	KLTPType;

			KLTTracker( size_t maxIters = 20 );

			void trackFeatures( std::vector<size_t> & trackedIndices,
							    std::vector<PoseType> & poses,
							    const std::vector<KLTPType*>& patches,
							    const Image& current );

			void trackMultiscale( std::vector<size_t> & trackedIndices,
								  std::vector<PoseType> & poses,
								  const std::vector<KLTPType*>& patches,
								  const std::vector<Image>& pyramid );

			float ssdThreshold() const { return _ssdThresh; }
			void  setSSDThreshold( float ssd ) { _ssdThresh = ssd; }

			/* percentage of pixels that need to project in current view */
			void  setMinPixFraction( float frac ){ _minPix = frac * Math::sqr( pSize ); }
			void  setEarlyStepOutAvgSSD( float ssd ){ _earlyStepOutThresh = ssd; }
			
			bool trackPatch( PoseType& pose, 
							 const KLTPType& patch,
							 const uint8_t* current, size_t currStride,
							 size_t width, size_t height );

		private:
			size_t _maxIters;
			float  _ssdThresh;
			size_t _minPix;
			float  _earlyStepOutThresh;
			
			bool checkBounds( const Eigen::Vector2f & p, size_t width, size_t height ) const
			{
				size_t h = pSize >> 1;
				if( p[ 0 ] <= h || p[ 1 ] <= h || p[ 0 ] + h >= width || p[ 1 ] + h >= height )
					return false;
				return true;
			}

	};


	template <class PoseType, size_t pSize>
	inline KLTTracker<PoseType, pSize>::KLTTracker( size_t maxIters ) :
		_maxIters( maxIters ),
		_ssdThresh( 0.0f )
	{
		setMinPixFraction( 0.9f );
		setEarlyStepOutAvgSSD( 10.0f );
		_ssdThresh = Math::sqr( 20.0f );
	}

	template <class PoseType, size_t pSize>
	inline void KLTTracker<PoseType, pSize>::trackFeatures( std::vector<size_t> & trackedIndices,
									std::vector<PoseType> & poses,
									const std::vector<KLTPType*>& patches,
									const Image& current )
	{
		size_t currStride;
		const uint8_t* currImgPtr = current.map( &currStride );

		size_t w = current.width();
		size_t h = current.height();	

		Eigen::Vector2f pp;
		Eigen::Vector2f p2( 0.0f, 0.0f );
		for( size_t i = 0; i < patches.size(); i++ ){
			// track each single feature
			PoseType & pose = poses[ i ];
			const KLTPType& patch = *patches[ i ];

		   	pose.transform( pp, p2 );
			if( !checkBounds( pp, w, h ) ){
				continue;
			}

			if( trackPatch( pose, patch, currImgPtr, currStride, w, h ) ){
				trackedIndices.push_back( i );
			}
		}

		current.unmap( currImgPtr );
	}

	template <class PoseType, size_t pSize>
	inline bool KLTTracker<PoseType, pSize>::trackPatch( PoseType & pose, 
									   					 const KLTPType& patch,
									   					 const uint8_t* current, size_t currStride,
									   					 size_t width, size_t height )
	{
		Eigen::Vector2f p2;
		Eigen::Vector2f pp;

		size_t iter = 0;

		Vector2f point;
		typename KLTPType::JacType jSum;
		typename PoseType::ParameterVectorType delta;
		float diffSum = 0.0f;
		size_t npix = 0;
		size_t numLines;
		while( iter < _maxIters ){
			jSum.setZero();
			numLines = pSize;
			diffSum = 0.0f;
			npix = 0;

			p2[ 1 ] = 0.0f;
			const typename KLTPType::JacType* J = patch.jacobians();
			const uint8_t* temp = patch.pixels();
			while( numLines-- ){
				p2[ 0 ] = 0.0f;//tempPos.x - halfSize; 
				for( size_t i = 0; i < pSize; i++ ){
					pose.transform( pp, p2 );
					if( ( size_t )pp[ 0 ] < width && ( size_t )pp[ 1 ] < height ){
						float deltaImg = ( int16_t )current[ ( size_t )pp[ 1 ] * currStride + ( size_t )pp[ 0 ] ] - ( int16_t )*temp;

						diffSum += ( deltaImg * deltaImg );

						jSum += ( *J *  deltaImg );
						npix++;
					}

					temp++;
					J++;
					p2[ 0 ] += 1;
				}
				p2[ 1 ] += 1;
			}
			
			if( npix < _minPix )
				return false;

			// solve for the delta:
			delta = patch.inverseHessian() * jSum;

			pose.applyInverse( -delta );

			/* early step out? */
			if( diffSum / npix < _earlyStepOutThresh )
				return true;
			
			iter++;
		}

		if( ( diffSum / npix ) > _ssdThresh ){
			return false;
		}
		return true;
	}

	template <class PoseType, size_t pSize>
	inline void KLTTracker<PoseType, pSize>::trackMultiscale( std::vector<size_t> & trackedIndices,
									  std::vector<PoseType> & poses,
									  const std::vector<KLTPType*>& patches,
									  const std::vector<Image>& pyramid )
	{
		int i = pyramid.size() - 1;
		float allScale   = ( float )pyramid[ i ].width() / pyramid[ 0 ].width();
		float interScale = ( float )pyramid[ i - 1 ].width() / pyramid[ i ].width();

		// map all:
		std::vector<const uint8_t*> ptr;
		std::vector<size_t>     stride;
		ptr.resize( pyramid.size() );
		stride.resize( pyramid.size() );
		for( size_t i = 0; i < pyramid.size(); i++ ){
			ptr[ i ] = pyramid[ i ].map( &stride[ i ] );
		}

		float origSSD = _ssdThresh;

		for( size_t p = 0; p < patches.size(); p++ ){
			const KLTPType & patch = *patches[ p ];

			// TODO: how do we need to change the pose correctly
			PoseType & currPose = poses[ p ];
			currPose.scale( allScale );

			i = pyramid.size() - 1;
			bool tracked = true;
			_ssdThresh = origSSD / allScale;
			while( i >= 0 ){
				tracked = trackPatch( currPose,
									  patch,
									  ptr[ i ],
									  stride[ i ],
									  pyramid[ i ].width(), 
									  pyramid[ i ].height() );
									  
				if( !tracked )
					break;
				if( i ){
					currPose.scale( interScale );
				}
				i--;
				_ssdThresh /= interScale;
			}
		
			if( tracked ){
				trackedIndices.push_back( p );
			}
			_ssdThresh = origSSD;
		}

		for( size_t i = 0; i < pyramid.size(); i++ ){
			pyramid[ i ].unmap( ptr[ i ] );
		}
	}
}

#endif
