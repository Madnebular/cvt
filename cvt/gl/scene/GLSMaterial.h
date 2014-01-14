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

#ifndef CVT_GLSMATERIAL_H
#define CVT_GLSMATERIAL_H

#include <cvt/gfx/Color.h>
#include <cvt/gl/scene/GLScene.h>
#include <cvt/gl/scene/GLSTexture.h>

#include <cvt/geom/scene/SceneMaterial.h>

namespace cvt {
	enum GLSMaterialFeatures {
		GL_SCENEMATERIAL_NONE			= ( 1 << 0 ),
		GL_SCENEMATERIAL_AMBIENT		= ( 1 << 1 ), // rgba ambient
		GL_SCENEMATERIAL_DIFFUSE		= ( 1 << 2 ),
		GL_SCENEMATERIAL_SPECULAR		= ( 1 << 3 ),
		GL_SCENEMATERIAL_DIFFUSE_MAP	= ( 1 << 4 ),
		GL_SCENEMATERIAL_AMBIENT_MAP	= ( 1 << 5 ),
		GL_SCENEMATERIAL_SPECULAR_MAP	= ( 1 << 6 ), // rgb = color, a = shininess
		GL_SCENEMATERIAL_NORMALMAP		= ( 1 << 7 )
	};

	CVT_ENUM_TO_FLAGS( GLSMaterialFeatures, GLSMaterialFlags )



	class GLSMaterial
	{
		public:
			GLSMaterial( const String& name );
			GLSMaterial( const SceneMaterial& mat, const GLScene& scene );
			~GLSMaterial();

			const Color& specularColor() const;
			void		 setSpecularColor( const Color& c );

			float		 shininess() const;
			void		 setShininess( float shininess );

			const Color& diffuseColor() const;
			void		 setDiffuseColor( const Color& c );

			const Color& ambientColor() const;
			void		 setAmbientColor( const Color& c );

			const String& name() const;

			const GLTexture*  diffuseMap() const;


			GLSMaterialFlags flags() const { return _flags; }
			GLSMaterialFlags& flags() { return _flags; }

		private:
			String			  _name;
			GLSMaterialFlags  _flags;
			Color			  _ambient;
			Color		      _diffuse;
			Color			  _specular;
			float			  _shininess;
			const GLSTexture* _ambientMap;
			const GLSTexture* _diffuseMap;
			const GLSTexture* _specMap;
			const GLSTexture* _normalMap;
	};

	inline GLSMaterial::GLSMaterial( const String& name ) : _name( name ), _flags( GL_SCENEMATERIAL_NONE ),
         _ambient( Color::WHITE ), _diffuse( Color::WHITE ), _specular( Color::WHITE ),
         _shininess( 1.0f ), _ambientMap( NULL ), _diffuseMap( NULL ), _specMap( NULL ), _normalMap( NULL )
	{
	}

	inline GLSMaterial::GLSMaterial( const SceneMaterial& mat, const GLScene& scene ) : _name( mat.name() ), _flags( GL_SCENEMATERIAL_NONE ),
         _ambient( Color::WHITE ), _diffuse( Color::WHITE ), _specular( Color::WHITE ),
        _shininess( 1.0f ), _ambientMap( NULL ), _diffuseMap( NULL ), _specMap( NULL ), _normalMap( NULL )
	{
		const GLSTexture* tex;

		if( mat.flags() & SCENEMATERIAL_AMBIENT ) {
			_flags |= GL_SCENEMATERIAL_AMBIENT;
			_ambient = mat.ambientColor();
		}

		if( mat.flags() & SCENEMATERIAL_DIFFUSE ) {
			_flags |= GL_SCENEMATERIAL_DIFFUSE;
			_diffuse = mat.diffuseColor();
		}

		if( mat.flags() & SCENEMATERIAL_SPECULAR ) {
			_flags |= GL_SCENEMATERIAL_SPECULAR;
			_specular = mat.specularColor();
			_shininess = mat.shininess();
		}

		if( mat.flags() & SCENEMATERIAL_AMBIENTMAP && ( tex = scene.texture( mat.ambientMap() ) ) != NULL ) {
			_flags |= GL_SCENEMATERIAL_AMBIENT_MAP;
			_ambientMap = tex;
		}

		if( mat.flags() & SCENEMATERIAL_DIFFUSEMAP && ( tex = scene.texture( mat.diffuseMap() ) ) != NULL) {
			_flags |= GL_SCENEMATERIAL_DIFFUSE_MAP;
			_diffuseMap = tex;
		}

		if( mat.flags() & SCENEMATERIAL_SPECULARMAP && ( tex = scene.texture( mat.specularMap() ) ) != NULL) {
			_flags |= GL_SCENEMATERIAL_SPECULAR_MAP;
			_specMap = tex;
		}

		if( mat.flags() & SCENEMATERIAL_NORMALMAP && ( tex = scene.texture( mat.normalMap() ) ) != NULL) {
			_flags |= GL_SCENEMATERIAL_NORMALMAP;
			_normalMap = tex;
		}

	}


	inline GLSMaterial::~GLSMaterial()
	{
	}

	inline const Color& GLSMaterial::specularColor() const
	{
		return _specular;
	}

	inline void GLSMaterial::setSpecularColor( const Color& c )
	{
		_specular = c;
		_flags |= GL_SCENEMATERIAL_SPECULAR;
	}

	inline float GLSMaterial::shininess() const
	{
		return _shininess;
	}

	inline void GLSMaterial::setShininess( float shininess )
	{
		_shininess = shininess;
	}

	inline const Color& GLSMaterial::diffuseColor() const
	{
		return _diffuse;
	}

	inline void GLSMaterial::setDiffuseColor( const Color& c )
	{
		_diffuse = c;
		_flags |= GL_SCENEMATERIAL_DIFFUSE;
	}

	inline const Color& GLSMaterial::ambientColor() const
	{
		return _ambient;
	}

	inline void GLSMaterial::setAmbientColor( const Color& c )
	{
		_ambient = c;
		_flags |= GL_SCENEMATERIAL_AMBIENT;
	}

	inline const String& GLSMaterial::name() const
	{
		return _name;
	}

	inline const GLTexture* GLSMaterial::diffuseMap() const
    {
        if( !_diffuseMap ) return NULL;
        return _diffuseMap->texture();
    }
}

#endif
