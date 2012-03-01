/*
			CVT - Computer Vision Tools Library

 	 Copyright (c) 2012, Philipp Heise, Sebastian Klose

 	THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 	PARTICULAR PURPOSE.
 */
#ifndef CVT_IWARP_H
#define CVT_IWARP_H

#include <cvt/gfx/IFilter.h>
#include <cvt/gfx/IMapScoped.h>
#include <cvt/util/Plugin.h>
#include <cvt/util/PluginManager.h>
#include <cvt/vision/CameraCalibration.h>

namespace cvt {
	class IWarp : public IFilter {
		public:
			IWarp();
			~IWarp();
			void apply( const ParamSet* attribs, IFilterType iftype ) const;

			static void apply( Image& dst, const Image& src, const Image& warp );

			static void warpTunnel( Image& dst, float radius, float cx, float cy );
			static void warpFishEye( Image& idst, float strength, float cx, float cy );
			static void warpUndistort( Image& idst, float k1, float k2, float cx, float cy, float fx, float fy, size_t srcWidth, size_t srcHeight, float k3 = 0, float p1 = 0, float p2 = 0 );


			/**
			  @brief Calculate the warp for each pixel to undistort an image aquired by a calibrated camera.
			  @param idst the warp image
			  @param Knew the new camera matrix for the undistorted image
			  @param srcWidth the width of the original image
			  @param srcHeight the height of the original image
			  @param alpha value between 0 ... 1 to get only valid pixels or all original pixels
			  @param beta value between 0 ... 1 to get only valids pixels or all pixels of the selected rectangle
			 */
			static void warpUndistort( Image& idst, Matrix3f& Knew, const CameraCalibration& calib, size_t srcWidth, size_t srcHeight, float alpha = 0.0f, float beta = 0.0f );

			/*
			   @brief Generic warp function
			   @param op a generic function object 'Vector2f function( const Vector2f& in )'
			 */
			template<typename TFUNC>
			static void warpGeneric( Image& idst, TFUNC op );

		private:
			static void applyFC1( Image& dst, const Image& src, const Image& warp );
			static void applyFC4( Image& dst, const Image& src, const Image& warp );
			static void applyU8C1( Image& dst, const Image& src, const Image& warp );
			static void applyU8C4( Image& dst, const Image& src, const Image& warp );

			IWarp( const IWarp& t );
	};

	template<typename TFUNC>
	inline void IWarp::warpGeneric( Image& idst, TFUNC op )
	{
		if( idst.format() != IFormat::GRAYALPHA_FLOAT )
			throw CVTException( "Unsupported warp image type" );

		float* dst;
		size_t w, h;

		IMapScoped<float> map( idst );
		w = idst.width();
		h = idst.height();

		for( size_t y = 0; y < h; y++ ) {
			dst = map.ptr();
			for( size_t x = 0; x < w; x++ ) {
				Vector2f p = op( Vector2f( x, y ) );
				*dst++ = Math::clamp<float>( p.x, 0, w - 1 );
				*dst++ = Math::clamp<float>( p.y, 0, h - 1 );
			}
			map++;
		}
	}
}

#endif
