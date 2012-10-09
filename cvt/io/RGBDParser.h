/*
            CVT - Computer Vision Tools Library

     Copyright (c) 2012, Philipp Heise, Sebastian Klose

    THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
    KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
    PARTICULAR PURPOSE.
 */
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

                Quaterniond orientation;
                Vector3d    position;

                bool        poseValid;
                double		stamp;

                template<typename T>
                Matrix4<T> pose() const
                {
                    Matrix4d mat = orientation.toMatrix4();
                    mat[ 0 ][ 3 ] = position.x;
                    mat[ 1 ][ 3 ] = position.y;
                    mat[ 2 ][ 3 ] = position.z;
                    return ( Matrix4<T> )mat;
                }
            };

            RGBDParser( const String& folder, double maxStampDiff = 0.05 );

            void loadNext();

            size_t				iter()    const { return _idx; }
            size_t				size()    const { return _stamps.size(); }            
            bool                hasNext() const { return _idx < _stamps.size(); }
            const RGBDSample&	data()    const { return _sample; }

            void                setIdx( size_t idx ) { _idx = idx; }

        private:
            const double			 _maxStampDiff;
            String					 _folder;

            std::vector<Quaterniond> _orientations;
            std::vector<Vector3d>    _positions;
            std::vector<String>		 _rgbFiles;
            std::vector<String>		 _depthFiles;
            std::vector<double>		 _stamps;
            std::vector<bool>		 _poseValid;

            RGBDSample				_sample;
            size_t					_idx;

            void loadGroundTruth();
            void loadRGBFilenames( std::vector<double> & stamps );
            void loadDepthFilenames( std::vector<double> & stamps );
            void loadDepthAndRGB( std::vector<double>& rgbStamps, std::vector<double>& depthStamps );

            bool readNext( Quaterniond& orientation, Vector3d& position, double& stamp, DataIterator& iter );

            bool readNextFilename( String& filename, double& stamp, DataIterator& iter );

            void sortOutData( const std::vector<double>& rgbStamps,
                              const std::vector<double>& depthStamps,
                              bool rgbAndDepthAssocitated );

            size_t findClosestMatchInGTStamps( double val, size_t startIdx );

            size_t findClosestMatchInDepthStamps( double stamp,
                                                  size_t dIdx,
                                                  const std::vector<double>& depth );
    };


}

#endif
