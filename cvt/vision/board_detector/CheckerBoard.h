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

#ifndef CVT_CHECKERBOARD_H
#define CVT_CHECKERBOARD_H

#include "IBoard.h"

class CheckerBoard: public IBoard {
public:
    CheckerBoard(float width,
                 float height,
                 float stepSize,
                 size_t maxSteps,
                 size_t totalPoints,
                 float maxEstimateError);
    virtual ~CheckerBoard() {}
    
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
     *         around the center by quadrant.
     * @return Pointset2f of diagonally arranged Point2f
     *
     *   1
     * 4 o 2
     *   3
     */
    const cvt::PointSet2f& directionsModel();
    
private:
    float _width;
    float _height;
    float _stepSize;
    size_t _maxSteps;
    size_t _totalPoints;
    float _maxEstimateError;
    cvt::PointSet2f _directionsModel;
};

CheckerBoard::CheckerBoard(float width,
                           float height,
                           float stepSize,
                           size_t maxSteps,
                           size_t totalPoints,
                           float maxEstimateError) :
_width(width),
_height(height),
_stepSize(stepSize),
_maxSteps(maxSteps),
_totalPoints(totalPoints),
_maxEstimateError(maxEstimateError)
{
    _directionsModel.clear();
    
    _directionsModel.add( cvt::Point2f( 0,  _stepSize ) );
    _directionsModel.add( cvt::Point2f(  _stepSize, 0 ) );
    _directionsModel.add( cvt::Point2f( 0, -_stepSize ) );
    _directionsModel.add( cvt::Point2f( -_stepSize, 0 ) );
}

void CheckerBoard::extractFeatures(cvt::PointSet2f &features,
                                   const cvt::Image &image)
{
    // TODO
}

inline const float CheckerBoard::width()
{
    return _width;
}

inline const float CheckerBoard::height()
{
    return _height;
}

inline const size_t CheckerBoard::maxSteps()
{
    return _maxSteps;
}

inline const float CheckerBoard::stepSize()
{
    return _stepSize;
}

inline const float CheckerBoard::maxEstimateError()
{
    return _maxEstimateError;
}

inline const size_t CheckerBoard::totalPoints()
{
    return _totalPoints;
}

inline const cvt::PointSet2f &CheckerBoard::directionsModel()
{
    return _directionsModel;
}


#endif
