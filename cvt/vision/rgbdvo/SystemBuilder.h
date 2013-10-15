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

#ifndef CVT_SYSTEMBUILDER_H
#define CVT_SYSTEMBUILDER_H

#include <cvt/vision/rgbdvo/RobustWeighting.h>
#include <cvt/math/Math.h>

namespace cvt {
    template <class EigenMat>
    static bool hasNaN( const EigenMat& mat )
    {
        for( int i = 0; i < mat.rows(); i++ ){
            for( int k = 0; k < mat.cols(); k++ ){
                if( Math::isNaN( mat( i, k ) ) ||
                    Math::isInf( mat( i, k ) ) ){
                    return true;
                }
            }
        }
        return false;
    }

    template <class LossFunc>
    class SystemBuilder
    {
        public:
            SystemBuilder( const LossFunc& lf ) : _lossFunc( lf )
            {
            }

            template <class HessType, class JType>
            float build( HessType& H,
                          JType& b,
                          const JType* jacobians,
                          const float* residuals,
                          size_t n ) const
            {
                // standard: robust lossfunc
                JType jtmp;
                b.setZero();
                H.setZero();

                float ssd = 0.0f;
                for( size_t i = 0; i < n; ++i ){
                    // compute the delta
                    ssd += Math::sqr( residuals[ i ] );

                    float weight = _lossFunc.weight( residuals[ i ] );
                    jtmp = weight * jacobians[ i ];
                    H.noalias() += jtmp.transpose() * jacobians[ i ];
                    b.noalias() += jtmp * residuals[ i ];

                }
                return ssd;
            }

        private:
            const LossFunc& _lossFunc;
    };

    // specialized implementation for non robust lossfunc
    template <>
    template <class HessType, class JType>
    inline float SystemBuilder<NoWeighting<float> >::build( HessType& H, JType& b,
                                                             const JType* jacobians,
                                                             const float* residuals,
                                                             size_t n ) const
    {
        // standard: robust lossfunc
        float ssd = 0.0f;
        b.setZero();
        H.setZero();
        for( size_t i = 0; i < n; ++i ){
            // compute the delta
            ssd += Math::sqr( residuals[ i ] );

            const JType& jtmp = jacobians[ i ];
            H.noalias() += jtmp.transpose() * jtmp;
            b.noalias() += jtmp * residuals[ i ];

        }
        return ssd;
    }

}

#endif // SYSTEMBUILDER_H
