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

#ifndef CVT_IMODEL_H
#define CVT_IMODEL_H

#include <cvt/geom/PointSet.h>
#include <cvt/gfx/Image.h>

class IBoard {
public:
  virtual void extractFeatures(cvt::PointSet2f& features,
                               const cvt::Image& image) =0;
  /**
   * @brief  Getter
   * @return width in cm
   */
  virtual const float width() =0;
  
  /**
   * @brief  Getter
   * @return height in cm
   */
  virtual const float height() =0;
  
  /**
   * @brief  Getter
   * @return stepSize in cm
   */
  virtual const float stepSize() =0;
  
  /**
   * @brief  Getter
   * @return maximum number of possible steps within board
   */
  virtual const size_t maxSteps() =0;
  
  /**
   * @brief  Getter
   * @return total points
   */
  virtual const size_t totalPoints() =0;
  
  /**
   * @brief  Getter
   * @return maximum error in cm
   */
  virtual const float maxEstimateError() =0;
  
  /**
   * @brief  Getter
   * @return Primitive search directions in model
   */
  virtual const cvt::PointSet2f& directionsModel() = 0;
};

#endif
