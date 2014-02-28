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

#include "BoardDetector.h"

namespace cvt {
  
  BoardDetector::BoardDetector(IBoard* board) :
    _board(board)
  {}
  
  BoardDetector::BoardDetector(const BoardDetector& other) :
    _board(other._board),
    _error(other._error),
    _matched(other._matched),
    _unmatched_data(other._unmatched_data),
    _homography(other._homography)
  {}
  
  bool BoardDetector::detect(const Image& image )
  {
    _board->extractFeatures( _unmatched_data, image );
    
    // Needs at least 5 features
    if ( _unmatched_data.size() < _board->totalPoints() / 2 )
    {
      return false;
    }
    
    bool success = ransacHomography( RANSAC_ITERATIONS, image );
    
    PointSet2f copy( _unmatched_data );
    setDiff( _unmatched_data, copy, _matched._data );
    
    return success;
  }
  
  bool BoardDetector::ransacHomography( size_t ransac_iterations,
                                       const Image& image )
  {
    _matched.clear();
    
    Point2f best_init;
    float best_error = FLT_MAX;
    TwoPointSet2f best_matched;
    
    if ( _unmatched_data.size() < 8 )
    {
      return false;
    }
    
    for ( size_t i = 0; i < ransac_iterations; ++i )
    {
      // Estimate rough homography from neighbors of random init point
      Point2f init = _unmatched_data[ rand() % _unmatched_data.size() ];
      
      // create multiple estimates per init point (up to 3)
      std::vector<Matrix3f> homography_estimates;
      roughHomography( homography_estimates, init );
      
      if( homography_estimates.size() < 1 )
      {
        continue;
      }
      
      for( size_t j = 0; j < homography_estimates.size(); ++j )
      {
        Matrix3f H = homography_estimates[ j ];
        
        PointSet2f unmatched_model( _unmatched_data );
        unmatched_model.transform( H.inverse() );
        TwoPointSet2f unmatched( _unmatched_data, unmatched_model );
        TwoPointSet2f matched;
        
        // Expand matched points along the four model directions
        // until border is reached
        expandToBorders( Point2f(0, 0), matched, unmatched );
        
        if ( matched.size() < 8 ) {
          continue;
        }
        
        // Grow matched points by recursively considering the point in its
        // neighborhood consistent with the current model
        grow( matched, unmatched );
        // Model space coordinates are relative to init point, init should
        // be in the top left corner
        bool landscape = setOriginTopLeft( matched._model );
        H = matched.alignPerspective();
        rematch( matched, landscape );
        
        if ( matched.size() < 8 ) {
          continue;
        }
        
        if ( !checkBorders( matched._model, landscape ) )
        {
          continue;
        }
        
        // Rate current model
        float error = calculateError( H, matched );
        
        if ( error < best_error )
        {
          best_error = error;
          best_matched = matched;
          best_init = init;
          
          if ( best_matched.size() == _board->totalPoints() ) {
            break;
          }
        }
      }
    }
    
    if (best_matched.size() < 4)
    {
      return false;
    }

    _matched = best_matched;
    _homography = _matched.alignPerspective();
    return true;
  }
  
  float BoardDetector::calculateError(const Matrix3f& H,
                                      const TwoPointSet2f& matched) const
  {
    //TODO: normalize error
    float total_error = ( _board->totalPoints() - matched.size() ) *
    UNMATCHED_PENALTY * _board->maxEstimateError();
    
    Matrix3f inverse = H.inverse();
    for (size_t i = 0; i < matched.size(); ++i )
    {
      Point2f expected = inverse * matched._data[ i ];
      total_error += ( expected - matched._model[ i ] ).length();
    }
    return total_error;
  }
  
  void BoardDetector::expandToBorders(const Point2f& init_model,
                                      TwoPointSet2f& matched,
                                      TwoPointSet2f& unmatched) const
  {
    PointSet2f directionsModel = _board->directionsModel();
    for( size_t d = 0; d < directionsModel.size(); ++d )
    {
      for( unsigned int counter = 1; counter < _board->maxSteps(); ++counter )
      {
        Point2f estimate( init_model + counter * directionsModel[ d ] );
        size_t idx = findClosestPoint( estimate, unmatched._model );
        Point2f actual( unmatched._model[ idx ] );
        
        if( ( estimate - actual ).length() < _board->maxEstimateError() )
        {
          matched.add( unmatched._data[ idx ], estimate );
        }
      }
    }
  }
  
  void BoardDetector::grow(TwoPointSet2f& matched,
                           TwoPointSet2f& unmatched) const
  {
    PointSet2f directionsModel( _board->directionsModel() );
    for( size_t d = 0; d < directionsModel.size(); ++d )
    {
      for ( size_t i = 0; i < matched.size(); ++i )
      {
        Point2f estimate( matched._model[ i ] + directionsModel[ d ] );
        size_t idx = findClosestPoint(estimate, unmatched._model);
        Point2f actual( unmatched._model[ idx ] );
        
        if ( ( estimate - actual ).length() < _board->maxEstimateError() &&
            !contains( matched._model, estimate ) )
        {
          matched.add( unmatched._data[ idx ], estimate );
        }
      }
    }
  }
  
  bool BoardDetector::setOriginTopLeft(PointSet2f& matched_model) const
  {
    float xmax = -FLT_MAX;
    float xmin = FLT_MAX;
    float ymax = -FLT_MAX;
    float ymin = FLT_MAX;
    
    for (size_t i = 0; i < matched_model.size(); ++i )
    {
      Point2f p = matched_model[ i ];
      if ( p.x < xmin )
      {
        xmin = p.x;
      }
      if ( p.x > xmax )
      {
        xmax = p.x;
      }
      if (p.y < ymin )
      {
        ymin = p.y;
      }
      if (p.y > ymax) {
        ymax = p.y;
      }
    }
    
    Point2f origin(xmin, ymin);
    
    // Transform all model points to be relative to the new origin
    for (size_t i = 0; i < matched_model.size(); ++i )
    {
      matched_model[ i ] -= origin;
    }
    
    if ( (xmax - xmin) < (ymax - ymin) ) {
      return false;
    } else {
      return true;
    }
  }
  
  void BoardDetector::rematch(TwoPointSet2f& matched, bool landscape) const
  {
    float horizontal = _board->width();
    float vertical = _board->height();
    if ( !landscape ) {
      float tmp = horizontal;
      horizontal = vertical;
      vertical = tmp;
    }
    
    // Points outside of current model
    TwoPointSet2f inside;
    for(size_t i = 0; i < matched.size(); ++i) {
      Point2f p( matched._model[ i ] );
      if ( p.x >= 0 && p.x <= horizontal &&
          p.y >= 0 && p.y <= vertical )
      {
        inside.add(matched._data[ i ], p);
      }
    }
    
    matched = inside;
  }
  
  bool BoardDetector::checkBorders(const PointSet2f &matched_model, 
                                   bool landscape) const
  {
    size_t left = 0;
    size_t right = 0;
    size_t top = 0;
    size_t bottom = 0;
    
    float horizontal = _board->width();
    float vertical = _board->height();
    if ( !landscape ) {
      float tmp = horizontal;
      horizontal = vertical;
      vertical = tmp;
    }
    
    for ( size_t i = 0; i < matched_model.size(); ++i ) {
      Point2f p = matched_model[ i ];
      if ( p.x == 0 )          left++;
      if ( p.x == horizontal ) right++;
      if ( p.y == 0 )          top++;
      if ( p.y == vertical )   bottom++;
    }
    
    return (left > 1) && (right > 1) && (top > 1) && (bottom > 1);
  }
  
  void BoardDetector::findNearestNeighbors(PointSet2f& neighbors,
                                           const Point2f& point,
                                           const PointSet2f& candidates,
                                           size_t number_of_neighbors) const
  {
    neighbors.clear();
    
    PointSet2f neighbors_and_point;
    neighbors_and_point.resize(number_of_neighbors+1);
    
    std::partial_sort_copy( candidates.begin(), candidates.end(),
                           neighbors_and_point.begin(),
                           neighbors_and_point.end(),
                           SquareDistanceComp(point) );
    
    for ( size_t i = 0; i < number_of_neighbors; ++i )
    {
      neighbors.add(neighbors_and_point[ i + 1 ]);
    }
  }
  
  size_t BoardDetector::findClosestPoint(const Point2f& point,
                                         const PointSet2f& candidates) const
  {
    float min_distance = FLT_MAX;
    
    size_t closest_idx;
    for ( size_t i = 0; i < candidates.size(); ++i )
    {
      float distance = ( candidates[ i ] - point ).length();
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }
    return closest_idx;
  }
  
  void BoardDetector::alignCClockwise(PointSet2f& points,
                                      const Point2f& point) const
  {
    std::sort( points.begin(), points.end(), AngleComp(point) );
  }
  
  void BoardDetector::roughHomography(std::vector<Matrix3f>& estimates,
                                      const Point2f& center) const
  {
    estimates.clear();
    
    //Find nearest neighbors of given point
    PointSet2f six_neighbors;
    findNearestNeighbors(six_neighbors, center, _unmatched_data, 6);
    
    //Find lines through center
    PointSet2f tabu;
    std::vector< std::pair< Point2f, Point2f > > line_points;
    for( size_t i = 0; i < six_neighbors.size(); ++i )
    {
      if( contains( tabu, six_neighbors[ i ] ) ) {
        continue;
      }
      //Line from point i through center
      Line2D<float> l = Line2D<float>( center, six_neighbors[ i ] );
      Line2D<float> normal;
      normal.setOrthogonal( l, center );
      
      Point2f p;
      bool found = false;
      for( size_t j = i + 1; !found && j < six_neighbors.size(); ++j )
      {
        if( contains( tabu, six_neighbors[ j ] ) ) {
          continue;
        }
        float distance = Math::abs( l.distance( six_neighbors[ j ] ) );
        //Other point must be on the other side of center
        if( normal.distance( six_neighbors[ j ] ) > 0 &&
           distance < _board->maxEstimateError() )
        {
          found = true;
          p = six_neighbors[ j ];
        }
      }
      if( found )
      {
        tabu.add( six_neighbors[ i ] );
        tabu.add( p );
        line_points.push_back( std::make_pair( six_neighbors[ i ], p ) );
        //                gfx_->drawLine(six_neighbors[i], p);
      }
    }
    
    if( line_points.size() > 1 ) {
      PointSet2f crossModel = _board->directionsModel();
      for( size_t i = 0; i < line_points.size() - 1; ++i )
      {
        for( size_t j = i + 1; j < line_points.size(); ++j )
        {
          PointSet2f four_neighbors;
          four_neighbors.add( line_points[ i ].first );
          four_neighbors.add( line_points[ i ].second );
          four_neighbors.add( line_points[ j ].first );
          four_neighbors.add( line_points[ j ].second );
          alignCClockwise( four_neighbors, center );
          estimates.push_back( crossModel.alignPerspective( four_neighbors ) );
        }
      }
    }
  }
  
  bool BoardDetector::contains(const PointSet2f& set, 
                               const Point2f& element) const
  {
    for ( size_t i = 0; i < set.size(); ++i )
    {
      if ( set[ i ] == element ) {
        return true;
      }
    }
    return false;
  }
  
  void BoardDetector::setDiff(PointSet2f& result,
                              const PointSet2f& A, 
                              const PointSet2f& B) const
  {
    result.clear();
    
    for (size_t i = 0; i < A.size(); ++i )
    {
      if ( !contains( B, A[ i ] ) ) {
        result.add( A[ i ] );
      }
    }
  }
  
  Matrix3f BoardDetector::homography() const
  {
    return _homography;
  }
  
  PointSet2f BoardDetector::unmatchedData() const
  {
    return _unmatched_data;
  }
  
  PointSet2f BoardDetector::matchedData() const
  {
    return _matched._data;
  }
  
  PointSet2f BoardDetector::matchedModel() const
  {
    return _matched._model;
  }
  
  void BoardDetector::drawLine(Image& image,
                               const Point2f& start,
                               const Point2f& end,
                               float r, float g, float b) const
  {
    GFXEngineImage gi( image );
    GFX gfx( &gi );
    gfx.color().set( r, g, b );
    gfx.drawLine(start, end);
  }
  
  /**
   * @brief Draws a cross (X) of given size and color
   * @param gfx GFX object for drawing
   * @param p Point2f to be drawn
   * @param size Diagonal size of cross in pixels
   * @param r red
   * @param g green
   * @param b blue
   */
  void BoardDetector::mark(Image& image, const Point2f& p, float size,
                           float r, float g, float b) const
  {
    GFXEngineImage gi( image );
    GFX gfx( &gi );
    gfx.color().set( r, g, b );
    
    gfx.drawLine( p + Vector2f(-size,0.0f), p + Vector2f(size,0.0f) );
    gfx.drawLine( p + Vector2f(0.0f,-size), p + Vector2f(0.0f,size) );
  }
  
  /**
   * @brief Marks all points in the PointSet2f with a cross of given size and color
   * @param g GFX object for drawing.
   * @param pts PointSet2f to be drawn
   * @param size Diagonal size of cross in pixels
   * @param r red
   * @param g green
   * @param b blue
   */
  void BoardDetector::mark(Image& image, const PointSet2f& pts, float size,
                           float r, float g, float b) const
  {
    GFXEngineImage gi( image );
    GFX gfx( &gi );
    gfx.color().set( r, g, b );
    
    for (size_t i = 0; i < pts.size(); ++i )
    {
      gfx.drawLine( pts[ i ] + Vector2f(-size,0.0f), pts[ i ] + Vector2f(size,0.0f) );
      gfx.drawLine( pts[ i ] + Vector2f(0.0f,-size), pts[ i ] + Vector2f(0.0f,size) );
    }
  }
  
  /**
   * @brief drawHomographyLines for all points in PointSet
   * @param g GFX object for drawing
   * @param pts
   */
  void BoardDetector::drawHomographyLines(Image& image,
                                          const PointSet2f& matched_model,
                                          const Matrix3f& homography) const
  {
    float xmax = -FLT_MAX;
    float xmin = FLT_MAX;
    float ymax = -FLT_MAX;
    float ymin = FLT_MAX;
    
    for (size_t i = 0; i < matched_model.size(); ++i ) {
      Point2f p = matched_model[ i ];
      if ( p.x < xmin )
      {
        xmin = p.x;
      }
      if ( p.x > xmax )
      {
        xmax = p.x;
      }
      if (p.y < ymin )
      {
        ymin = p.y;
      }
      if (p.y > ymax) {
        ymax = p.y;
      }
    }
    
    
    float horizontal;
    float vertical;
    if ( (xmax - xmin) < (ymax - ymin) ) {
      horizontal = _board->height();
      vertical = _board->width();
    } else {
      horizontal = _board->width();
      vertical = _board->height();
    }
    
    Point2f top_left = homography * Point2f( 0, 0 );
    Point2f top_right = homography * Point2f( horizontal, 0 );
    Point2f bottom_left = homography * Point2f( 0, vertical );
    Point2f bottom_right = homography * Point2f( horizontal, vertical );
    
    GFXEngineImage gi( image );
    GFX g( &gi );
    g.color().set( GREEN );
    g.drawLine(top_left, top_right);
    g.color().set( RED );
    g.drawLine(top_left, bottom_left);
    g.color().set( BLUE );
    g.drawLine(top_right, bottom_right);
    g.drawLine(bottom_right, bottom_left);
  }
  
  
  void BoardDetector::saveDebugImages(const Image& image,
                                      TwoPointSet2f& unmatched,
                                      TwoPointSet2f& matched,
                                      Matrix3f& homography,
                                      Point2f& init,
                                      int j) const
  {
    Image copy(image);
    
    PointSet2f current_unmatched_data;
    setDiff(current_unmatched_data, _unmatched_data, matched._data);
    mark(copy, unmatched._data, 10.0f, RED);
    mark(copy, matched._data, 10.0f, GREEN );

    
    Point2f init_model = homography.inverse() * init;
    PointSet2f ps = _board->directionsModel();
    mark(copy, homography * init_model, 10.0f, WHITE);
    drawLine(copy,
             homography * (init_model + ps[0]),
             homography * (init_model + ps[2]),
             CYAN );
    drawLine(copy,
             homography * (init_model + ps[1]),
             homography * (init_model + ps[3]),
             MAGENTA );
    drawHomographyLines(copy, matched._model, homography);
    
    String path( "/Users/mwis/Developer/cvt_board_detection/img/debug/bd_" );
    path.sprintfConcat("%d.png", j);
    copy.save(path);
  }
  
}
