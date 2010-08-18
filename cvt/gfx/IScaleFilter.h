#ifndef ISCALEFILTER_H
#define ISCALEFILTER_H

#include <string>

namespace cvt {

	struct IConvolveAdaptiveSize {
		size_t numw;
		ssize_t incr;
	};

	struct IConvolveAdaptivef {
		IConvolveAdaptiveSize* size;
		float* weights;
	};


	class IScaleFilter {
		public:
			IScaleFilter( float support = 1.0f, float sharpsmooth = 0.0f ) : _support( support ), _sharpsmooth( sharpsmooth ) {};
			virtual float eval( float x ) const = 0;
			virtual const std::string name() const = 0;
			IConvolveAdaptivef* getAdaptiveConvolutionWeights( size_t dst, size_t src, size_t& maxsupport, bool nonegincr = true ) const;

		protected:
			float _support;
			float _sharpsmooth;
	};

	class IScaleFilterBilinear : public IScaleFilter
	{
			IScaleFilterBilinear( float sharpsmooth = 0.0f ) : IScaleFilter( 1.0f, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "Bilinear"; };
	};

	class IScaleFilterCubic : public IScaleFilter
	{
			IScaleFilterCubic( float sharpsmooth = 0.0f ) : IScaleFilter( 2.0f, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "Cubic"; };
	};

	class IScaleFilterCatmullRom : public IScaleFilter
	{
			IScaleFilterCatmullRom( float sharpsmooth = 0.0f ) : IScaleFilter( 2.0f, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "CatmullRom"; };
	};

	class IScaleFilterMitchell : public IScaleFilter
	{
			IScaleFilterMitchell( float sharpsmooth = 0.0f ) : IScaleFilter( 2.0f, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "Mitchell"; };
	};

	class IScaleFilterLanczos : public IScaleFilter
	{
			IScaleFilterLanczos( float support = 3.0f, float sharpsmooth = 0.0f ) : IScaleFilter( support, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "Lanczos"; };
	};

	class IScaleFilterBlackman : public IScaleFilter
	{
			IScaleFilterBlackman( float support = 3.0f, float sharpsmooth = 0.0f ) : IScaleFilter( support, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "Blackman"; };
	};

	class IScaleFilterBlackmanHarris : public IScaleFilter
	{
			IScaleFilterBlackmanHarris( float support = 3.0f, float sharpsmooth = 0.0f ) : IScaleFilter( support, sharpsmooth ) {};
			virtual float eval( float x ) const ;
			virtual const std::string name() const { return "BlackmanHarris"; };
	};


};


#endif
