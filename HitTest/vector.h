/*
 * File:	vector.h
 * Author:	Benjamin Russig
 *
 * Created on 05.09.2013 19:53:25
 */

#ifndef __HTEST_VECTOR_H__
#define __HTEST_VECTOR_H__


// Library setup
#include "_config.h"



//////
//
// Dependencies
//

// C++ STL
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	#include <utility>
#endif
#include <cmath>

// Local includes
#include "utils.h"



// Open library namespace
namespace htest {



//////
//
// Structs
//

/** @brief A POD-type struct representing a 3-dimensional vector. */
template <class flt_type>
struct HTEST_API SVector3
{
	////
	// Data members

	/** @brief Supported aliases for the vector data. */
	union
	{
		struct {flt_type x, y, z;};
		struct {flt_type u, v, w;};
		struct {flt_type r, g, b;};
		flt_type c [3];
	};

	/** @brief Read-only null vector value. */
	static const SVector3 null_vec;

	/** @brief Read-only X-axis unit vector value. */
	static const SVector3 x_unit_vec;

	/** @brief Read-only Y-axis unit vector value. */
	static const SVector3 y_unit_vec;

	/** @brief Read-only Z-axis unit vector value. */
	static const SVector3 z_unit_vec;


	////
	// Operators

	/** @brief Implicit conversion to an array of floating point numbers. */
	inline operator flt_type* (void)
	{
		return c;
	}

	/** @brief Implicit conversion to an array of const floating point numbers. */
	inline operator const flt_type* (void) const
	{
		return c;
	}

	inline SVector3 operator + (const SVector3& other) const
	{
		SVector3 r;
		r.x = x + other.x;
		r.y = y + other.y;
		r.z = z + other.z;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline SVector3&& operator + (SVector3&& other) const
	{
		other.x += x;
		other.y += y;
		other.z += z;
		return std::move(other);
	}
	inline friend SVector3&& operator + (SVector3&& v1, const SVector3& v2)
	{
		v1.x += v2.x;
		v1.y += v2.y;
		v1.z += v2.z;
		return std::move(v1);
	}
	inline friend SVector3&& operator + (SVector3&& v1, SVector3&& v2)
	{
		v2.x += v1.x;
		v2.y += v1.y;
		v2.z += v1.z;
		return std::move(v2);
	}
#endif

	inline SVector3 operator - (const SVector3& other) const
	{
		SVector3 r;
		r.x = x - other.x;
		r.y = y - other.y;
		r.z = z - other.z;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline SVector3&& operator - (SVector3&& other) const
	{
		other.x = x - other.x;
		other.y = y - other.y;
		other.z = z - other.z;
		return std::move(other);
	}
	inline friend SVector3&& operator - (SVector3&& v1, const SVector3& v2)
	{
		v1.x -= v2.x;
		v1.y -= v2.y;
		v1.z -= v2.z;
		return std::move(v1);
	}
	inline friend SVector3&& operator - (SVector3&& v1, SVector3&& v2)
	{
		v1.x -= v2.x;
		v1.y -= v2.y;
		v1.z -= v2.z;
		return std::move(v1);
	}
#endif

	inline SVector3& operator += (const SVector3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	inline SVector3& operator -= (const SVector3& other)
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	inline SVector3 operator - (void) const
	{
		SVector3 r;
		r.x = -x;
		r.y = -y;
		r.z = -z;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector3&& operator - (SVector3&& v)
	{
		v.x = -v.x;
		v.y = -v.y;
		v.z = -v.z;
		return std::move(v);
	}
#endif

	inline SVector3 operator * (flt_type scalar) const
	{
		SVector3 r;
		r.x = x * scalar;
		r.y = y * scalar;
		r.z = z * scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector3&& operator * (SVector3&& v, flt_type scalar)
	{
		v.x *= scalar;
		v.y *= scalar;
		v.z *= scalar;
		return std::move(v);
	}
#endif
	inline friend SVector3 operator * (flt_type scalar, const SVector3& v)
	{
		SVector3 r;
		r.x = v.x * scalar;
		r.y = v.y * scalar;
		r.z = v.z * scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector3&& operator * (flt_type scalar, SVector3&& v)
	{
		v.x *= scalar;
		v.y *= scalar;
		v.z *= scalar;
		return std::move(v);
	}
#endif
	inline flt_type operator * (const SVector3& other) const
	{
		return x*other.x + y*other.y + z*other.z;
	}

	inline SVector3 operator / (flt_type scalar) const
	{
		SVector3 r;
		r.x = x / scalar;
		r.y = y / scalar;
		r.z = z / scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector3&& operator / (SVector3&& v, flt_type scalar)
	{
		v.x /= scalar;
		v.y /= scalar;
		v.z /= scalar;
		return std::move(v);
	}
#endif

	inline SVector3& operator *= (flt_type scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		return *this;
	}
	inline SVector3& operator /= (flt_type scalar)
	{
		x /= scalar;
		y /= scalar;
		z /= scalar;
		return *this;
	}

	/**
	 * @brief
	 *		Equality comparison. WARNING - not robust with regards to floating point
	 *		stability!
	 */
	inline bool operator == (const SVector3& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}

	/**
	 * @brief
	 *		Unequality comparison. WARNING - not robust with regards to floating point
	 *		stability!
	 */
	inline bool operator != (const SVector3& other) const
	{
		return x != other.x || y != other.y || z != other.z;
	}

	/**
	 * @brief
	 *		Spatial less-than comparison. WARNING - not robust with regards to floating
	 *		point stability!
	 */
	inline bool operator < (const SVector3& other) const
	{
		return x < other.x && y < other.y && z < other.z;
	}

	/**
	 * @brief
	 *		Spatial less-than-or-equal comparison. WARNING - not robust with regards to
	 *		floating point stability!
	 */
	inline bool operator <= (const SVector3& other) const
	{
		return x <= other.x && y <= other.y && z <= other.z;
	}

	/**
	 * @brief
	 *		Spatial greater-than comparison. WARNING - not robust with regards to
	 *		floating point stability!
	 */
	inline bool operator > (const SVector3& other) const
	{
		return x > other.x && y > other.y && z > other.z;
	}

	/**
	 * @brief
	 *		Spatial greater-than-or-equal comparison. WARNING - not robust with regards
	 *		to floating point stability!
	 */
	inline bool operator >= (const SVector3& other) const
	{
		return x >= other.x && y >= other.y && z >= other.z;
	}


	////
	// Methods

	inline SVector3& zeros (void)
	{
		z = y = x = 0.0;
		return *this;
	}
	inline SVector3& zeroes (void)
	{
		z = y = x = 0.0;
		return *this;
	}

	inline flt_type length (void) const
	{
		return std::sqrt(x*x + y*y + z*z);
	}
	inline flt_type length_sqr (void) const
	{
		return x*x + y*y + z*z;
	}

	inline SVector3& normalize (void)
	{
		flt_type len = length();
		x /= len;
		y /= len;
		z /= len;
		return *this;
	}

	inline SVector3 cross (const SVector3& other) const
	{
		SVector3 r;
		r.x = y*other.z - z*other.y;
		r.y = z*other.x - x*other.z;
		r.z = x*other.y - y*other.x;
		return r;
	}

	inline static SVector3 get_null_vec (void)
	{
		SVector3 null_vec = {{{0.0, 0.0, 0.0}}};
		return null_vec;
	}


	////
	// Setters / Getters

	inline SVector3& set_all(flt_type scalar)
	{
		x = scalar;
		y = scalar;
		z = scalar;
		return *this;
	}
	inline SVector3& set_xyz(flt_type x, flt_type y, flt_type z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		return *this;
	}
	inline SVector3& set_uvw(flt_type u, flt_type v, flt_type w)
	{
		this->u = u;
		this->v = v;
		this->w = w;
		return *this;
	}
	inline SVector3& set_rgb(flt_type r, flt_type g, flt_type b)
	{
		this->r = r;
		this->g = g;
		this->b = b;
		return *this;
	}
};


/** @brief A POD-type struct representing 4-dimensional and homogeneous 3D vectors. */
template <class flt_type>
struct HTEST_API SVector4
{
	////
	// Data members

	/** @brief Supported aliases for the vector data. */
	union
	{
		struct {flt_type x, y, z, w;};
		struct {flt_type r, g, b, a;};
		flt_type c [4];
	};

	/** @brief Read-only null vector value. */
	static const SVector4 null_vec;

	/** @brief Read-only homogenous 3D null vector value. */
	static const SVector4 null_vec_h;

	/** @brief Read-only X-axis unit vector value. */
	static const SVector4 x_unit_vec;

	/** @brief Read-only Y-axis unit vector value. */
	static const SVector4 y_unit_vec;

	/** @brief Read-only Z-axis unit vector value. */
	static const SVector4 z_unit_vec;

	/** @brief Read-only W-axis unit vector value. */
	static const SVector4 w_unit_vec;

	/** @brief Read-only X-axis homogenous 3D unit vector value. */
	static const SVector4 x_unit_vec_h;

	/** @brief Read-only Y-axis homogenous 3D unit vector value. */
	static const SVector4 y_unit_vec_h;

	/** @brief Read-only Z-axis homogenous 3D unit vector value. */
	static const SVector4 z_unit_vec_h;


	////
	// Operators

	/** @brief Implicit conversion to an array of floating point numbers. */
	inline operator flt_type* (void)
	{
		return c;
	}

	/** @brief Implicit conversion to an array of const floating point numbers. */
	inline operator const flt_type* (void) const
	{
		return c;
	}

	/** @brief Explicit conversion to a @ref htest::SVector3 . */
	inline SVector3<flt_type>& to_vec3 (void)
	{
		return *((SVector3<flt_type>*)c);
	}

	/** @brief Explicit conversion to a const @ref htest::SVector3 . */
	inline const SVector3<flt_type>& to_vec3 (void) const
	{
		return *((SVector3<flt_type>*)c);
	}

	inline SVector4 operator + (const SVector4& other) const
	{
		SVector4 r;
		r.x = x + other.x;
		r.y = y + other.y;
		r.z = z + other.z;
		r.w = w + other.w;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline SVector4&& operator + (SVector4&& other) const
	{
		other.x += x;
		other.y += y;
		other.z += z;
		other.w += w;
		return std::move(other);
	}
	inline friend SVector4&& operator + (SVector4&& v1, const SVector4& v2)
	{
		v1.x += v2.x;
		v1.y += v2.y;
		v1.z += v2.z;
		v1.w += v2.w;
		return std::move(v1);
	}
	inline friend SVector4&& operator + (SVector4&& v1, SVector4&& v2)
	{
		v2.x += v1.x;
		v2.y += v1.y;
		v2.z += v1.z;
		v2.w += v1.w;
		return std::move(v2);
	}
#endif

	inline SVector4 operator - (const SVector4& other) const
	{
		SVector4 r;
		r.x = x - other.x;
		r.y = y - other.y;
		r.z = z - other.z;
		r.z = w - other.w;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline SVector4&& operator - (SVector4&& other) const
	{
		other.x = x - other.x;
		other.y = y - other.y;
		other.z = z - other.z;
		other.w = w - other.w;
		return std::move(other);
	}
	inline friend SVector4&& operator - (SVector4&& v1, const SVector4& v2)
	{
		v1.x -= v2.x;
		v1.y -= v2.y;
		v1.z -= v2.z;
		v1.w -= v2.w;
		return std::move(v1);
	}
	inline friend SVector4&& operator - (SVector4&& v1, SVector4&& v2)
	{
		v1.x -= v2.x;
		v1.y -= v2.y;
		v1.z -= v2.z;
		v1.w -= v2.w;
		return std::move(v1);
	}
#endif

	inline SVector4& operator += (const SVector4& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		w += other.w;
		return *this;
	}
	inline SVector4& operator -= (const SVector4& other)
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
		w -= other.w;
		return *this;
	}

	inline SVector4 operator - (void) const
	{
		SVector4 r;
		r.x = -x;
		r.y = -y;
		r.z = -z;
		r.w = -w;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector4&& operator - (SVector4&& v)
	{
		v.x = -v.x;
		v.y = -v.y;
		v.z = -v.z;
		v.w = -v.w;
		return std::move(v);
	}
#endif

	inline SVector4 operator * (flt_type scalar) const
	{
		SVector4 r;
		r.x = x * scalar;
		r.y = y * scalar;
		r.z = z * scalar;
		r.w = w * scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector4&& operator * (SVector4&& v, flt_type scalar)
	{
		v.x *= scalar;
		v.y *= scalar;
		v.z *= scalar;
		v.w *= scalar;
		return std::move(v);
	}
#endif
	inline friend SVector4 operator * (flt_type scalar, const SVector4& v)
	{
		SVector4 r;
		r.x = v.x * scalar;
		r.y = v.y * scalar;
		r.z = v.z * scalar;
		r.w = v.w * scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector4&& operator * (flt_type scalar, SVector4&& v)
	{
		v.x *= scalar;
		v.y *= scalar;
		v.z *= scalar;
		v.w *= scalar;
		return std::move(v);
	}
#endif
	inline flt_type operator * (const SVector4& other) const
	{
		return x*other.x + y*other.y + z*other.z + w*other.w;
	}

	inline SVector4 operator / (flt_type scalar) const
	{
		SVector4 r;
		r.x = x / scalar;
		r.y = y / scalar;
		r.z = z / scalar;
		r.w = w / scalar;
		return r;
	}
#ifdef HTEST_CPP11_HAS_RVALUE_REFS
	inline friend SVector4&& operator / (SVector4&& v, flt_type scalar)
	{
		v.x /= scalar;
		v.y /= scalar;
		v.z /= scalar;
		v.w /= scalar;
		return std::move(v);
	}
#endif

	inline SVector4& operator *= (flt_type scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return *this;
	}
	inline SVector4& operator /= (flt_type scalar)
	{
		x /= scalar;
		y /= scalar;
		z /= scalar;
		w /= scalar;
		return *this;
	}

	/**
	 * @brief
	 *		Equality comparison. WARNING - not robust with regards to floating point
	 *		stability!
	 */
	inline bool operator == (const SVector4& other) const
	{
		return x == other.x && y == other.y && z == other.z && w == other.w;
	}

	/**
	 * @brief
	 *		Unequality comparison. WARNING - not robust with regards to floating point
	 *		stability!
	 */
	inline bool operator != (const SVector4& other) const
	{
		return x != other.x || y != other.y || z != other.z || w != other.w;
	}

	/**
	 * @brief
	 *		Spatial less-than comparison. WARNING - not robust with regards to floating
	 *		point stability!
	 */
	inline bool operator < (const SVector4& other) const
	{
		return x < other.x && y < other.y && z < other.z && w < other.w;
	}

	/**
	 * @brief
	 *		Spatial less-than-or-equal comparison. WARNING - not robust with regards to
	 *		floating point stability!
	 */
	inline bool operator <= (const SVector4& other) const
	{
		return x <= other.x && y <= other.y && z <= other.z && w <= other.w;
	}

	/**
	 * @brief
	 *		Spatial greater-than comparison. WARNING - not robust with regards to
	 *		floating point stability!
	 */
	inline bool operator > (const SVector4& other) const
	{
		return x > other.x && y > other.y && z > other.z && w > other.w;
	}

	/**
	 * @brief
	 *		Spatial greater-than-or-equal comparison. WARNING - not robust with regards
	 *		to floating point stability!
	 */
	inline bool operator >= (const SVector4& other) const
	{
		return x >= other.x && y >= other.y && z >= other.z && w >= other.w;
	}


	////
	// Methods

	inline SVector4& zeros (void)
	{
		w = z = y = x = 0.0;
		return *this;
	}
	inline SVector4& zeroes (void)
	{
		w = z = y = x = 0.0;
		return *this;
	}
	inline SVector4& zeros_h (void)
	{
		z = y = x = 0.0; w = 1.0;
		return *this;
	}
	inline SVector4& zeroes_h (void)
	{
		z = y = x = 0.0; w = 1.0;
		return *this;
	}

	inline flt_type length (void) const
	{
		return std::sqrt(x*x + y*y + z*z + w*w);
	}
	inline flt_type length_h (void) const
	{
		return std::sqrt(x*x + y*y + z*z) / w;
	}

	inline flt_type length_sqr (void) const
	{
		return x*x + y*y + z*z + w*w;
	}
	inline flt_type length_h_sqr (void) const
	{
		return (x*x + y*y + z*z) / (w*w);
	}

	inline SVector4& normalize (void)
	{
		flt_type len = length();
		x /= len;
		y /= len;
		z /= len;
		w /= len;
		return *this;
	}
	inline SVector4& normalize_h (void)
	{
		flt_type len = length_h();
		x /= len;
		y /= len;
		z /= len;
		return *this;
	}
	inline SVector4& normalize_hp (void)
	{
		flt_type len = length_h();
		x /= w*len;
		y /= w*len;
		z /= w*len;
		w = 1.0;
		return *this;
	}

	inline SVector3<flt_type> cross_h (const SVector3<flt_type>& other) const
	{
		SVector3<flt_type> r;
		r.x = y*other.z - z*other.y;
		r.y = z*other.x - x*other.z;
		r.z = x*other.y - y*other.x;
		return r;
	}

	inline SVector3<flt_type> cross_hp (const SVector3<flt_type>& other) const
	{
		SVector3<flt_type> r;
		flt_type xp = x/w, yp = y/w, zp = z/w;
		r.x = yp*other.z - zp*other.y;
		r.y = zp*other.x - xp*other.z;
		r.z = xp*other.y - yp*other.x;
		return r;
	}

	inline void project (void)
	{
		x /= w;
		y /= w;
		z /= w;
		w = 1.0;
	}

	inline static SVector4 getNullVec (void)
	{
		SVector4 null_vec = {{{0.0, 0.0, 0.0, 0.0}}};
		return null_vec;
	}
	inline static SVector4 getNullVec_h (void)
	{
		SVector4 null_vec_h = {{{0.0, 0.0, 0.0, 1.0}}};
		return null_vec_h;
	}


	////
	// Setters / Getters

	inline SVector4& set_all(flt_type scalar)
	{
		x = scalar;
		y = scalar;
		z = scalar;
		w = scalar;
		return *this;
	}
	inline SVector4& set_xyzw(flt_type x, flt_type y, flt_type z, flt_type w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
		return *this;
	}
	inline SVector4& set_rgba(flt_type r, flt_type g, flt_type b, flt_type a)
	{
		this->r = r;
		this->g = g;
		this->b = b;
		this->a = a;
		return *this;
	}
};



// Close library namespace
}


#endif // ifndef __HTEST_VECTOR_H__
