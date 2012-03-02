/* 
 * File:   HomographyStitching.h
 * Author: sebi
 *
 * Created on February 27, 2012, 2:57 PM
 */

#ifndef CVT_HOMOGRAPHYSTITCHING_H
#define	CVT_HOMOGRAPHYSTITCHING_H

#include <cvt/math/Matrix.h>
#include <cvt/gfx/Image.h>
#include <cvt/vision/ORB.h>
#include <vector>
#include "CompositingSurface.h"


namespace cvt {

    class HomographyStitching {
        public:
            HomographyStitching();
            ~HomographyStitching();

            void addImage( const Image& img );

            const Image& stitchedImage() const { return _surface->compositedImage(); }
            
        private:
			CompositingSurface* _surface;
			size_t	_octaves;
			float	_scale;
			uint8_t	_fastCornerThreshold;
			size_t	_maxNumFeatures;
			bool	_nonMaxSuppress;
			float	_maxDescriptorDistance;

            std::vector<Image>		_images;
			std::vector<Matrix3f>	_homographies;
			std::vector< std::vector<ORBFeature> > _featuresForView;

			bool checkHomography( const Matrix3f& h );
			void addFeatures( const ORB& orb );
    };

}

#endif	

