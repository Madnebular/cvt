#ifndef CVT_FEATURE2D_H
#define CVT_FEATURE2D_H

#include <cvt/math/Vector.h>

namespace cvt {
	template<typename T>
	struct Feature2D {
		Feature2D( T x, T y, float angle = 0.0f, size_t octave = 0 );

		Vector2<T> pt;
		float angle;
		size_t octave;
	};

    template <typename T>
	inline Feature2D<T>::Feature2D( T x, T y, float a, size_t o ) : pt( x, y ), angle( a ), octave( o )
	{
	}

	typedef Feature2D<int> Feature2Di;
	typedef Feature2D<float> Feature2Df;
}

#endif
