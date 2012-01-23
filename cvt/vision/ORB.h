#ifndef CVT_ORB_H
#define CVT_ORB_H

#include <cvt/gfx/Image.h>
#include <cvt/vision/IntegralImage.h>
#include <cvt/vision/Feature2D.h>
#include <cvt/vision/BinaryDescriptor.h>
#include <cvt/util/SIMD.h>
#include <vector>
#include <iostream>
#include <iomanip>

namespace cvt {

	struct ORBFeature : public Feature2Df {
        ORBFeature( float x, float y, float angle = 0.0f, size_t octave = 0, float score = 0.0f ) :
            Feature2Df( x, y, angle, octave, score )
        {
        }

		size_t distance( const ORBFeature& f ) const
		{
			return SIMD::instance()->hammingDistance( desc.data(), f.desc.data(), desc.size() );
		}

		BinaryDescriptor<32> desc; // 256 bit vector
	};


	static inline std::ostream& operator<<( std::ostream& out, const ORBFeature& feature )
	{
		out << "( " << feature.pt.x << " , " << feature.pt.y << " ) Orientation: " << Math::rad2Deg( feature.angle );
        out << " Score: " << feature.score;
		out << "\nDescriptor: 0x";
		for( int i = 0; i < 32; i++ )
			out << std::hex << std::setfill( '0' ) << std::setw( 2 ) << (int)feature.desc.data()[ i ];
		out << std::endl;
		return out;
	}


	class ORB {
		friend bool _centroidAngleTest();
		public:
			ORB( const Image& img, size_t octaves = 3, float scalefactor = 0.5f, uint8_t cornerThreshold = 25, size_t numFeatures = 1000, bool nms = true );
			ORB( size_t octaves = 3, float scalefactor = 0.5f, uint8_t cornerThreshold = 25, size_t numFeatures = 1000, bool nms = true );
            ~ORB();

			void update( const Image & img );
			size_t size() const;
			const ORBFeature& operator[]( size_t index ) const;

		private:
			void detect( const Image& img, size_t octave );
            void extract( size_t octaves );

			void centroidAngle( ORBFeature& feature, const float* ptr, size_t widthstep );

			void descriptor( ORBFeature& feature, const float* ptr, size_t widthstep );

            void nonmaxSuppression( const std::vector<ORBFeature> & features );

			void selectBestFeatures( size_t num );

			typedef std::vector<ORBFeature> ContainerType;
			ContainerType	_features;

            IntegralImage*	_iimages;
			size_t			_octaves;
            float			_scaleFactor;
            float*			_scaleFactors;
			size_t			_currentOctave;

            // for OFAST
            uint8_t			 _threshold;
			size_t			 _numFeatures;
            bool             _nms;

			static const int _patterns[ 30 ][ 512 ][ 2 ];
			static const int _circularoffset[ 31 ];

            // border where we do not detect features:
            // 17+2 17->maximum test coord within patch + 2 for the integral image access
            static const int _border = 22;
	};

	inline size_t ORB::size() const
	{
		return _features.size();
	}

	inline const ORBFeature& ORB::operator[]( size_t index ) const
	{
		return _features[ index ];
	}
}

#endif
