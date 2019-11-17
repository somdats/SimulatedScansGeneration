
#ifndef __EIGEN_TOOLS_H__
#define __EIGEN_TOOLS_H__


//////
//
// Includes
//

// Eigen library
#include <Eigen/Dense>

// Local config
#include "_config.h"



//////
//
// Classes
//

/**
 * @brief
 *		Templated abstraction to both single and double precision versions of Eigen's
 *		data type shortcuts.
 *
 * This might be useful if the internal structure of the Eigen library changes some day
 * due to some major overhaul. When that happens, the shortcut names are more likely to
 * stay unchanged and still be located in their old place than the template classes
 * themselves.
 */
template <class flt_type>
struct APISPEC EigenTypes
{
};

/** @brief Single precision specialization of @ref #EigenTypes . */
template<>
struct APISPEC EigenTypes<float>
{
	typedef Eigen::Vector2f Vec2;
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Vector4f Vec4;
	typedef Eigen::Matrix4f Mat4;
};

/** @brief Double precision specialization of @ref #EigenTypes . */
template<>
struct APISPEC EigenTypes<double>
{
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	typedef Eigen::Vector4d Vec4;
	typedef Eigen::Matrix4d Mat4;
};


#endif // ifndef __EIGEN_TOOLS_H__
