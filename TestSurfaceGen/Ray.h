
#ifndef __RAY_H__
#define __RAY_H__


//////
//
// Includes
//

// API config
#include "_config.h"

// Local includes
#include "EigenTools.h"



//////
//
// Classes
//

/**
 * @brief
 *		POD-type struct representing a ray in 3D space, defined via origin and direction.
 */
template <class flt_type>
struct APISPEC Ray
{
	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;

	/** @brief Homogenous 3D vector type. */
	typename typedef EigenTypes<real>::Vec4 Vec4;


	////
	// Data members

	/** @brief Homogenous position vector of ray origin. */
	Vec4 origin;

	/** @brief Normalized ray direction vector. */
	Vec3 direction;


	////
	// Methods

	/**
	 * @brief Calculates the point on the ray that is @c t units away from the origin.
	 */
	inline Vec3 point3At (real t) const
	{
		return std::move(
			  Vec3(origin.x()/origin.w(), origin.y()/origin.w(), origin.z()/origin.w())
			+ t*direction
		);
	}

	/**
	 * @brief
	 *		Calculates the point (as homogenous position vector) on the ray that is @c t
	 *		units away from the origin.
	 */
	inline Vec4 point4At (real t) const
	{
		Vec3 point = std::move(point3At(t));
		return std::move(Vec4(point.x(), point.y(), point.z(), 1));
	}
};


#endif // ifndef __RAY_H__
