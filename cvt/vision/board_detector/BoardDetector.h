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

#ifndef CVT_BOARDDETECTOR_H
#define CVT_BOARDDETECTOR_H

#include <set>
#include <cvt/geom/Line2D.h>
#include <cvt/geom/PointSet.h>
#include <cvt/gfx/GFXEngineImage.h>
#include <cvt/geom/Line2D.h>

#include "IBoard.h"

//#define BOXFILTER 10.0f
//#define ADAPTIVE_THRESHOLD 0.5f

#define RED 1.0f, 0.0f, 0.0f
#define GREEN 0.0f, 1.0f, 0.0f
#define BLUE 0.0f, 0.0f, 1.0f
#define CYAN 0.0f, 1.0f, 1.0f
#define MAGENTA 1.0f, 0.0f, 1.0f
#define YELLOW 1.0f, 1.0f, 0.0f
#define WHITE 1.0f, 1.0f, 1.0f

#define UNMATCHED_PENALTY 200.0f
#define RANSAC_ITERATIONS 200
namespace cvt{
  
  typedef PointSet2f::iterator PsIter;
  typedef PointSet2f::const_iterator c_PsIter;
  
  class BoardDetector {
  public:
    BoardDetector(IBoard* board);
    BoardDetector( const BoardDetector& other );
    ~BoardDetector() {}
    
    
    /**
     * @brief  Detects board of given type (specified by _board) in the image
     *         and computes the pose of the board relative to the camera.
     *         After detection _matched._data will contain all feature points
     *         in the image belonging to the board, _matched._model will the
     *         corresponding points in model space, _unmatched_data the
     *         outliers and _homography the pose estimation
     * @param  image Image with board
     * @return True if board was successfully detected in given image,
     *         false otherwise
     */
    bool detect( const Image& image );
    
    /**
     * @brief  Getter
     * @return Pose estimation after detect returned true
     */
    Matrix3f homography() const;
    
    /**
     * @brief  Getter
     * @return Outliers after detect returned true
     */
    PointSet2f unmatchedData() const ;
    
    /**
     * @brief  Getter
     * @return Detected board points in the image
     */
    PointSet2f matchedData() const;
    
    /**
     * @brief  Getter
     * @return Detected board points in model space
     */
    PointSet2f matchedModel() const;
    
    /**
     * @brief  Debug function, draws a colored x at point p
     */
    void mark(Image &image, const Point2f& p, float size,
              float r, float g, float b) const;
    
    /**
     * @brief  Debug function, draws colored x at all points in pts
     */
    void mark(Image &image, const PointSet2f &pts, float size,
              float r, float g, float b) const;
    
    /**
     * @brief  Debug function, draws a projected rectangle to mark the board.
     * @param  image         Image in which to draw
     * @param  matched_model Model to determine orientation
     * @param  homography    Homography representing board position
     */
    void drawHomographyLines(Image& image,
                             const PointSet2f& matched_model,
                             const Matrix3f& homography) const;
    
    /**
     * @brief  Debug function, draws a colored line from start to end
     */
    void drawLine(Image& image,
                  const Point2f& start, const Point2f& end,
                  float r, float g, float b) const;
    
  private:
    /**
     * @brief  Compares points according to distance from given center point
     */
    struct SquareDistanceComp
    {
      Point2f _pt;
      SquareDistanceComp( const Point2f& pt) : _pt(pt)
      {
      }
      
      inline bool operator()(const Point2f& a, const Point2f& b) {
        return (a - _pt).lengthSqr() < (b - _pt).lengthSqr();
      }
    };
    
    /**
     * @brief  Compares two points according to the angle of its polar
     *        coordinates relative to a given center point
     */
    struct AngleComp
    {
      Point2f _pt;
      AngleComp(const Point2f& pt): _pt(pt)
      {
      }
      
      inline bool operator()(const Point2f& a, const Point2f& b)
      {
        return Math::atan2(a.y - _pt.y, a.x - _pt.x) <
        Math::atan2(b.y - _pt.y, b.x - _pt.x);
      }
    };
    
    /**
     * Struct for handling two PointSets in sync
     */
    struct TwoPointSet2f
    {
      TwoPointSet2f() {}
      
      TwoPointSet2f(const PointSet2f& data, const PointSet2f& model) :
        _data(data),
        _model(model)
      {}
      
      TwoPointSet2f(const TwoPointSet2f& other):
        _data(other._data),
        _model(other._model)
      {}
      
      ~TwoPointSet2f() {}
      
      inline void add(Point2f& data_point, Point2f& model_point)
      {
        _data.add(data_point);
        _model.add(model_point);
      }
      
      inline size_t size() const
      {
        return _data.size();
      }
      
      inline Matrix3f alignPerspective() const
      {
        return _model.alignPerspective(_data);
      }
      
      inline void clear()
      {
        _data.clear();
        _model.clear();
      }
      
      PointSet2f _data;
      PointSet2f _model;
    };
    
    /**
     * @brief  Uses ransac to find an optimal model (homography). For every
     *         iteration starts with a random init point, esimates rough
     *         homographies according to that point's neighborhood and tries
     *         to match as many points in _unmatched_data with that model as
     *         possible. Every iteration is evaluated using a least squares
     *         error.
     * @remarks The inliers from best estimate are used to estimate 
     *          the final homography, which is then saved to the 
     *          class variable_homography. 
     *          The inliers are saved in _matched.
     * @remarks For a successful estimation the following criteria have to 
     *          be met:
     *          - At least 8 points in _unmatched_data
     *          - 2 Points on the top, bottom, left and right border 
     *              respectively
     *          
     * @param  ransac_iterations Fixed number of iterations
     * @param  image             To draw in for debug purposes
     * @return                   True if homography was estimated successfully,
     *                           false otherwise
     */
    bool ransacHomography(size_t ransac_iterations, const Image& image );
    
    /**
     * @brief  Rates the passed model, i.e., the reprojection error in model 
     *         space between estimated and actual points in model space. 
     *         Penalizes unmatched points by a constant factor.
     * @param  H       Homography from model to data
     * @param  matched Inliers according to passed homography and _board
     * @return         Reprojection error
     */
    float calculateError(const Matrix3f& H,
                         const TwoPointSet2f& matched) const;
    
    /**
     * @brief  Beginning from the starting point searches along the model 
     *         directions for neighbors for n steps. Number of steps defined by
     *         _board->maxSteps. Maximum allowed estimate error defined by
     *         _board_->maxEstimateError.
     * @param  init_model Starting point
     * @param  matched    Found points will be added here
     * @param  unmatched  Search space
     */
    void expandToBorders(const Point2f& init_model,
                         TwoPointSet2f &matched, TwoPointSet2f &unmatched) const;
    
    /**
     * @brief  For every point in matched tries to expand along the four
     *         model directions. Maximum allowed estimate error defined by
     *         _board_->maxEstimateError.
     * @param  matched   Initial inliers. Will be expanded
     * @param  unmatched Search space. Will be reduced
     */
    void grow(TwoPointSet2f& matched,
              TwoPointSet2f& unmatched) const;
    
    /**
     * @brief  Finds the board's top left corner and changes the given model
     *         coordinates to be relative to that point. Additionaly determines
     *         the board orientation (landscape or portrait)
     * @param  matched_model Model points with undefined origin
     * @return               True if board is in landscape orientation,
     *                       false if portrait
     */
    bool setOriginTopLeft(PointSet2f& matched_model ) const;
    
    /**
     * @brief  Removes false positive inliers that are outside the model
     *         border
     * @param  matched   Inliers to be reduced
     * @param  landscape Whether false positive removal should be handled in
     *                   landscape or portrait mode
     */
    void rematch(TwoPointSet2f &matched, bool landscape) const;
    
    /**
     * @brief  Checks whether at least two points on every model border
     *         were found (two for top/bottom/left/right respectively)
     * @param  matched_model Inliers to be checked
     * @param  landscape     Wether check should be handled in landscape or
     *                       portrait mode
     * @return               True if model has at least two points on every
     *                       border
     */
    bool checkBorders(const PointSet2f& matched_model, bool landscape) const;
    
    /**
     * @brief  Search for n nearest neighbors
     * @param  neighbors           Will contain the neighbors afterwards
     * @param  point               Reference point
     * @param  candidates          Search space for nearest neighbor search
     * @param  number_of_neighbors Final size of neighbors PointSet
     */
    void findNearestNeighbors(PointSet2f& neighbors,
                              const Point2f& point,
                              const PointSet2f& candidates,
                              size_t number_of_neighbors) const;
    
    /**
     * @brief  Linear seach to find the index of a point from the 
     *         candidates that is closest to the given point
     * @param  point      Reference point
     * @param  candidates Search space
     * @return            Index of nearest point
     */
    size_t findClosestPoint(const Point2f& point,
                            const PointSet2f &candidates) const;
    
    /**
     * @brief  Sorts a PointSet of 4 points ascending according to the angle
     *         of their polar coordinates relative to the given point
     * @param  points PointSet to be sorted
     * @param  point  Reference point
     */
    inline void alignCClockwise(PointSet2f& points, 
                                const Point2f& point) const;
    
    /**
     * @brief  Estimates homographies from the 6 neighbors of the given center.
     *         6 neighbors are used to increase robustness against perspective
     *         deformations
     * @param  estimates Contains up to three estimates
     * @param  center    Origin for the homography estimates
     */
    void roughHomography(std::vector<Matrix3f>& estimates,
                         const Point2f& center) const;
    
    /**
     * @brief  Simple contains function for PointSet2f
     */
    bool contains(const PointSet2f& set, const Point2f& element) const;
    
    /**
     * @brief  Set difference (A\B)
     * @return A\B
     */
    void setDiff(PointSet2f& result,
                 const PointSet2f& A,
                 const PointSet2f& B) const;
    
    /**
     * @brief  Debug function to save an image of the current state
     * @param  image      Current state will be drawn in copy of this image
     * @param  unmatched  Outliers
     * @param  matched    Inliers
     * @param  homography Homography esimate
     * @param  init       Start point
     * @param  j          Ransac iteration
     */
    void saveDebugImages(const Image& image,
                         TwoPointSet2f& unmatched,
                         TwoPointSet2f& matched,
                         Matrix3f& homography,
                         Point2f& init,
                         int j) const;
    
    IBoard* _board;
    float _error;
    TwoPointSet2f _matched;
    PointSet2f _unmatched_data;
    Matrix3f _homography;
  };
}


#endif
