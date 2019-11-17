
#ifndef __SCANNABLE_OBJECT_H__
#define __SCANNABLE_OBJECT_H__


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
// Structs
//

/** @brief Struct representing a ray-surface intersection result. */
template <class flt_type>
struct APISPEC Intesection
{
	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;


	////
	// Data members

	/** @brief Does this instance describe an actual ray-surface hit? */
	bool hit;

	/** @brief Ray parameter at intersection. */
	real t;

	/** @brief 3D position vector of the intersection. */
	Vec3 pos;

	/** @brief Surface normal at the intersection. */
	Vec3 normal;
};



//////
//
// Interfaces
//

/** @brief Interface for solid objects that support ray intersection. */
template <class flt_type>
class APISPEC IScannableObject
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 2D vector type. */
	typename typedef EigenTypes<real>::Vec2 Vec2;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;

	/** @brief 3D Homogenous vector type. */
	typename typedef EigenTypes<real>::Vec4 Vec4;

	/** @brief 3D homogenous matrix type. */
	typename typedef EigenTypes<real>::Mat4 Mat4;


	////
	// Object construction / destruction

	/** @brief Virtual destructor (causes vtable creation). */
	virtual ~IScannableObject();


	////
	// Methods

	/**
	* @brief
	*		Tests for intersection with the given ray and reports information about the
	*		intersection (if any) that is closest to the ray origin.
	*/
	virtual Intesection<real> intersectRay (const Ray<real> &ray) const = 0;


	////
	// Setters / getters

	/** @brief Sets the world coordinate system transformation matrix for the object. */
	virtual void setWorldTransform (const Mat4 &mat) = 0;
	/** @brief Returns the current world coordinate system transformation matrix. */
	virtual Mat4 getWorldTransform (void) const = 0;
	/**
	 * @brief
	 *		Writes the current world coordinate system transformation matrix to @c out .
	 */
	virtual void getWorldTransform (Mat4 *out) const = 0;
};



#endif // ifndef __SCANNABLE_OBJECT_H__
