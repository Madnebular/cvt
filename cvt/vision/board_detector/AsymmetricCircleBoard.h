/*
 The MIT License (MIT)
 
 Copyright (c) 2011 - 2013, Philipp Heise and Sebastian Klose
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#ifndef CVT_ASYMMETRICCIRCLEBOARD_H
#define CVT_ASYMMETRICCIRCLEBOARD_H

#include "IBoard.h"
#include <cvt/gfx/IComponents.h>
#include <cvt/gfx/ifilter/Canny.h>
#include <cvt/geom/Ellipse.h>

#define CANNY_LOW 0.05f
#define CANNY_HIGH 0.3f

class AsymmetricCircleBoard: public IBoard {
public:
  AsymmetricCircleBoard( float width,
                        float height,
                        float stepSize,
                        size_t maxSteps,
                        size_t totalPoints,
                        float maxEstimateError );
  AsymmetricCircleBoard( const AsymmetricCircleBoard& other );
  virtual ~AsymmetricCircleBoard() {}
  
  /**
   * @brief  Performs canny filter to get edges, fits ellipses to components
   *         and then extracts the points of interest (ellipse centers)
   * @param  features PointSet of extracted ellipse centers
   *                  (will be cleared)
   * @param  image    Image from which to extract ellipses
   *                  (will be converted to greyscale)
   */
  void extractFeatures(cvt::PointSet2f &features,
                       const cvt::Image &image);
  const float width();
  const float height();
  const float stepSize();
  const size_t maxSteps();
  const size_t totalPoints();
  const float maxEstimateError();
  
  /**
   * @brief  Returns a Pointset2f of points aligned in a diagonal cross
   *         around the center (clockwise).
   * @return Pointset2f of diagonally arranged Point2f
   *
   * 1   2
   *   o
   * 4   3
   */
  const cvt::PointSet2f& directionsModel();
  
private:
  float           _width;
  float           _height;
  float           _stepSize;
  size_t          _maxSteps;
  size_t          _totalPoints;
  float           _maxEstimateError;
  cvt::PointSet2f _directionsModel;
};

AsymmetricCircleBoard::AsymmetricCircleBoard( float width,
                                             float height,
                                             float stepSize,
                                             size_t maxSteps,
                                             size_t totalPoints,
                                             float maxEstimateError ) :
_width(width),
_height(height),
_stepSize(stepSize),
_maxSteps(maxSteps),
_totalPoints(totalPoints),
_maxEstimateError(maxEstimateError)
{
  float step_half = sqrtf( ( stepSize * stepSize ) / 2.0 );
  
  _directionsModel.clear();
  _directionsModel.add(cvt::Point2f(-step_half, -step_half));
  _directionsModel.add(cvt::Point2f(step_half, -step_half));
  _directionsModel.add(cvt::Point2f(step_half, step_half));
  _directionsModel.add(cvt::Point2f(-step_half, step_half));
}

AsymmetricCircleBoard::AsymmetricCircleBoard( const AsymmetricCircleBoard& other ) :
_width(other._width),
_height(other._height),
_stepSize(other._stepSize),
_maxSteps(other._maxSteps),
_totalPoints(other._totalPoints),
_maxEstimateError(other._maxEstimateError)
{
}

void AsymmetricCircleBoard::extractFeatures( cvt::PointSet2f& features,
                                            const cvt::Image& image )
{
  features.clear();
  
  cvt::Image greyscale;
  // Convert im to grayscale
  image.convert( greyscale, cvt::IFormat::GRAY_FLOAT );
  // Extract edges
  cvt::Canny canny;
  canny.apply( greyscale, greyscale, CANNY_LOW, CANNY_HIGH );
  // extract components
  cvt::IComponents< float > comps( greyscale );
  
  for( size_t c = 0; c < comps.size(); ++c )
  {
    // Try to fit ellipse to extracted component
    cvt::Ellipsef ellipse;
    comps[ c ].fitEllipse( ellipse );
    
    // center coordinates have to be numbers
    if ( !cvt::Math::isNaN( ellipse.center().x ) &&
        !cvt::Math::isNaN( ellipse.center().y ) )
    {
      features.add( ellipse.center() );
    }
  }
}

inline const float AsymmetricCircleBoard::width()
{
  return _width;
}

inline const float AsymmetricCircleBoard::height()
{
  return _height;
}

inline const float AsymmetricCircleBoard::stepSize()
{
  return _stepSize;
}

inline const size_t AsymmetricCircleBoard::maxSteps()
{
  return _maxSteps;
}

inline const size_t AsymmetricCircleBoard::totalPoints()
{
  return _totalPoints;
}

inline const float AsymmetricCircleBoard::maxEstimateError()
{
  return _maxEstimateError;
}

inline const cvt::PointSet2f& AsymmetricCircleBoard::directionsModel()
{
  return _directionsModel;
}

#endif
