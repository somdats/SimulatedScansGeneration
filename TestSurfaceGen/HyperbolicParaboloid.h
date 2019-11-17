
#ifndef __HYPERBOLIC_PARABOLOID_H__
#define __HYPERBOLIC_PARABOLOID_H__


//////
//
// Includes
//

// API config
#include "_config.h"

// Local includes
#include "EigenTools.h"
#include "IScannableObject.h"



//////
//
// Interfaces
//

/**
 * @brief
 *		@link IScannableObject Scannable @endlink hyperbolic paraboloid algebraic surface
 *		(also known as a saddle function when in explicit form) with bounds.
 */
template <class flt_type>
class APISPEC HyperbolicParaboloid : public IScannableObject<flt_type>
{

public:

	////
	// Types

	/** @brief Real number type. */
	typename typedef IScannableObject<flt_type>::real real;

	/** @brief 2D vector type. */
	typename typedef IScannableObject<flt_type>::Vec2 Vec2;

	/** @brief 3D vector type. */
	typename typedef IScannableObject<flt_type>::Vec3 Vec3;

	/** @brief 3D Homogenous vector type. */
	typename typedef IScannableObject<flt_type>::Vec4 Vec4;

	/** @brief 3D homogenous matrix type. */
	typename typedef IScannableObject<flt_type>::Mat4 Mat4;


protected:

	////
	// Data members

	/** @brief Vector of the x and y coefficients. */
	Vec2 coeffs;

	/** @brief Vector of the x and y scaling factors. */
	Vec2 scale;

	/** @brief Vector storing x0 and y0 offsets. */
	Vec2 offset;

	/** @brief Minimum point of rectangular domain. */
	Vec2 rect_min;

	/** @brief Maximum point of rectangular domain. */
	Vec2 rect_max;

	/**
	 * @brief
	 *		Affine transformation modeling the object's position and orientation in
	 *		world space.
	 */
	Mat4 worldTransform;

	/** @brief The inverse of @ref #worldTransform . */
	Mat4 worldTransformInv;


public:

	////
	// Object construction / destruction

	/**
	 * @brief
	 *		Builds the paraboloid with given parameters.
	 *
	 * For the meaning of the parameters, consider the explicit representation of the
	 * hyperbolic paraboloid to be of the form z = cx (sx x - x0)² + cy (sy y - y0)²,
	 * with values x ranging from xmin to xmax and values y ranging from ymin to ymax
	 */
	HyperbolicParaboloid(real cx, real cy, real sx, real sy, real x0, real y0,
	                     real xmin=-1, real ymin=-1, real xmax=1, real ymax=1);

	/** @brief The destructor. */
	virtual ~HyperbolicParaboloid();


	////
	// Methods

	/**
	 * @brief
	 *		Tests for intersection with the given ray and returns the ray parameter t at
	 *		the intersection point (if any) that is closest to the ray origin.
	 */
	virtual bool intersectRay(real *t, const Ray<real> &ray) const;


	////
	// Setters / getters

	/** @brief Sets @ref #worldTransform . */
	virtual void setWorldTransform(const Mat4 &mat);
	/** @brief Returns current contents of @ref #worldTransform . */
	virtual Mat4 getWorldTransform(void) const;
	/** @brief Writes current contents of @ref #worldTransform to matrix @c out . */
	virtual void getWorldTransform(Mat4 *out) const;
};



#endif // ifndef __HYPERBOLIC_PARABOLOID_H__
