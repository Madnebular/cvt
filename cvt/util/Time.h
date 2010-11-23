#ifndef CVT_TIME_H
#define CVT_TIME_H

#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#ifdef APPLE
	#include <mach/mach_time.h>
#else
	#ifndef _POSIX_TIMERS
		#error "Posix timers not supported"
	#endif
	#ifndef _POSIX_MONOTONIC_CLOCK
		#error "Posix monotonic clock not supported"
	#endif
#endif

namespace cvt {

	class Time {
		public:
			Time();
			Time( const Time& t );
			~Time();
			void reset();
			double elapsedSeconds() const;
			double elapsedMilliSeconds() const;
			double elapsedMicroSeconds() const;
			double ms() const;
			double operator+( const Time& t ) const;
			double operator-( const Time& t ) const;
			double operator+( double ms ) const;
			double operator-( double ms ) const;
			const Time& operator+=( size_t ms );
			int compare( const Time& t ) const;

		private:
			double timespecToMS( const struct timespec& ts ) const;
			double timespecToUS( const struct timespec& ts ) const;
			void updateTimespec( struct timespec& ts ) const;

			struct timespec _ts;

#ifdef APPLE
			static mach_timebase_info_data_t _machTimebase;
#endif
	};

	inline Time::Time()
	{
#ifdef APPLE
		if( _machTimebase.denom == 0 )
               mach_timebase_info( &_machTimebase );
#endif
		reset();
	}

	inline Time::Time( const Time& t )
	{
		_ts.tv_sec = t._ts.tv_sec;
		_ts.tv_nsec = t._ts.tv_nsec;
	}

	inline Time::~Time() {}

	inline void Time::reset()
	{
		updateTimespec( _ts );
	}

	inline void Time::updateTimespec( struct timespec& ts ) const
	{
#ifdef APPLE
		uint64_t t = mach_absolute_time();
		uint64_t ns = t * ( _machTimebase.numer / _machTimebase.denom );
		ts.tv_sec = ns / 1000000000L;
        ts.tv_nsec = ns - (ts.tv_sec * 1000000000L);
#else
		clock_gettime( CLOCK_MONOTONIC, &ts );
#endif
	}

	inline double Time::timespecToMS( const struct timespec& ts ) const
	{
		return ( ( double ) ts.tv_sec ) * 1000.0 + ( ( double ) ts.tv_nsec ) * 0.000001;
	}

	inline double Time::timespecToUS( const struct timespec& ts ) const
	{
		return ( ( double ) ts.tv_sec ) * 1000000.0 + ( ( double ) ts.tv_nsec ) * 0.001;
	}

	inline double Time::elapsedSeconds() const
	{
		struct timespec ts2;
		updateTimespec( ts2 );
		return ( double ) ts2.tv_sec - ( double ) _ts.tv_sec;
	}

	inline double Time::elapsedMilliSeconds() const
	{
		struct timespec ts2;
		updateTimespec( ts2 );
		return timespecToMS( ts2 ) - timespecToMS( _ts );
	}

	inline double Time::elapsedMicroSeconds() const
	{
		struct timespec ts2;
		updateTimespec( ts2 );
		return timespecToUS( ts2 ) - timespecToUS( _ts );
	}

	inline double Time::operator-( const Time& t ) const
	{
		return timespecToMS( _ts ) - timespecToMS( t._ts );
	}

	inline double Time::operator+( const Time& t ) const
	{
		return timespecToMS( _ts ) + timespecToMS( t._ts );
	}

	inline double Time::operator-( double ms ) const
	{
		return timespecToMS( _ts ) - ms;
	}

	inline double Time::operator+( double ms ) const
	{
		return timespecToMS( _ts ) + ms;
	}

	inline double Time::ms() const
	{
		return timespecToMS( _ts );
	}

	inline const Time& Time::operator+=( size_t ms )
	{
		long ns;
		ldiv_t res;
		ns = ms * 1000000L + _ts.tv_nsec;
		res = ldiv( ns, 1000000000L );
		_ts.tv_sec += res.quot;
		_ts.tv_nsec = res.rem;
		return *this;
	}

	inline int Time::compare( const Time& t ) const
	{
		if ( _ts.tv_sec < t._ts.tv_sec)
			return -1;
		if ( _ts.tv_sec > t._ts.tv_sec)
			return 1;
		return ( int ) ( _ts.tv_nsec - t._ts.tv_nsec );
	}
}

#endif
