
#ifndef __PROJECTOR_H__
#define __PROJECTOR_H__


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
 *		Class modeling a ray-casting projector with simple intrinsic parameters moving
 *		around in 3D space.
 */
template <class flt_type>
class APISPEC Projector
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


protected:

	////
	// Data members

	/** @brief Affine transformation modeling the projector's position and orientation. */
	Mat4 viewMatrix;

	/** @brief The inverse of @ref #viewMatrix . */
	Mat4 viewMatrixInv;

	/** @brief Transformation modeling the projector's intrinsic parameters. */
	Mat4 projectionMatrix;

	/** @brief The inverse of @ref #projectionMatrix . */
	Mat4 projectionMatrixInv;

	/** @brief Vertical and horizontal ray-casting resolution. */
	Vec2 resolution;

	/**
	 * @brief
	 *		Sample aspect ratio. In combination with the @ref #resolution this determines
	 *		the overal projector aspect ratio. A default of 1 is assumed unless otherwise
	 *		specified.
	 */
	real SAR;


public:

	////
	// Object construction / destruction

	/** @brief Default constructor. Initializes the resolution to 100² and SAR to 1. */
	Projector();

	/** @brief Constructs the projector with the given parameters. */
	Projector(real resX, real resY, real SAR=1);

	/** @brief Constructs the projector with the given parameters. */
	Projector(const Vec2 &resolution, real SAR=1);

	/** @brief The destructor. */
	virtual ~Projector();


	////
	// Methods

	/** @brief Casts a ray at the given sample position. */
	virtual Ray<real> castRay (real posX, real posY) const;

	/** @brief Casts a ray at the given sample position. */
	inline Ray<real> castRay (const Vec2 &xyPos) const
	{
		return std::move(castRay(xyPos.x(), xyPos.y()));
	}

	/**
	 * @brief
	 *		Sets @ref #viewMatrix (and by extension, @ref #viewMatrixInv ) so that it
	 *		models the projector being at position @c pos, looking at a point @target,
	 *		with the vertical direction being aligned along the vector @c up .
	 */
	virtual void applyLookAt (const Vec4 &pos, const Vec4 &target, const Vec3 &up);

	/**
	* @brief
	*		Sets @ref #projectionMatrix (and by extension, @ref #projectionMatrixInv ) so
	*		that it models a projection with given FoV (in degrees), near and far planes.
	*/
	virtual void applyFovNearFar(real FoV, real near, real far);


	////
	// Setters / getters

	/** @brief Sets @ref #viewMatrix . */
	virtual void setViewMatrix (const Mat4 &mat);
	/** @brief Returns current contents of @ref #viewMatrix . */
	virtual Mat4 getViewMatrix (void) const;
	/** @brief Writes current contents of @ref #viewMatrix to matrix @c out . */
	virtual void getViewMatrix (Mat4 *out) const;

	/** @brief Sets @ref #projectionMatrix . */
	virtual void setProjectionMatrix (const Mat4 &mat);
	/** @brief Returns current contents of @ref #projectionMatrix . */
	virtual Mat4 getProjectionMatrix (void) const;
	/** @brief Writes current contents of @ref #projectionMatrix to matrix @c out . */
	virtual void getProjectionMatrix (Mat4 *out) const;

	/** @brief Sets @ref #resolution . */
	virtual void setResolution (const Vec2 &res);
	/** @brief Returns current contents of @ref #resolution . */
	virtual Vec2 getResolution (void) const;
	/** @brief Writes current contents of @ref #resolution to vector @c out . */
	virtual void getResolution (Vec2 *out) const;

	/** @brief Sets @ref #SAR . */
	virtual void setSAR (real value);
	/** @brief Returns current @ref #SAR . */
	virtual real getSAR (void) const;
};


#endif // ifndef __PROJECTOR_H__
