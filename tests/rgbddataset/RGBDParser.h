#ifndef CVT_RGBDPARSER_H
#define CVT_RGBDPARSER_H

#include <cvt/util/String.h>
#include <cvt/util/Data.h>
#include <cvt/util/DataIterator.h>
#include <cvt/io/FileSystem.h>
#include <cvt/gfx/Image.h>
#include <cvt/math/Matrix.h>

namespace cvt
{
	class RGBDParser
	{
		public:
			struct RGBDSample
			{
				Image		rgb;
				Image		depth;
				Matrix4d	pose;
				double		stamp;
			};

			RGBDParser( const String& folder );

			void loadNext();
			
			size_t				iter() const { return _idx; }
			size_t				size() const { return _stamps.size(); }
			const RGBDSample&	data() const { return _sample; }

		private:
			const double			_maxStampDiff;
			String					_folder;

			std::vector<Matrix4d>	_groundTruthPoses;
			std::vector<String>		_rgbFiles;
			std::vector<String>		_depthFiles;
			std::vector<double>		_stamps;

			RGBDSample				_sample;
			size_t					_idx;

			void loadGroundTruth();
			void loadRGBFilenames( std::vector<double> & stamps );
			void loadDepthFilenames( std::vector<double> & stamps );

			bool readNext( Matrix4d& pose, double& stamp, DataIterator& iter );
			bool readNextFilename( String& filename, double& stamp, DataIterator& iter );

			void sortOutData( const std::vector<double>& rgbStamps, 
							  const std::vector<double>& depthStamps );
	
			size_t findClosestMatchInGTStamps( double val, size_t startIdx );

			size_t findClosestMatchInDepthStamps( double stamp,
												  size_t dIdx,
												  const std::vector<double>& depth ); 
	};
			
			
}

#endif