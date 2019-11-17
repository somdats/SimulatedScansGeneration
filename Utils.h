
#ifndef __UTILS_H__
#define __UTILS_H__


//////
//
// Includes
//

// Local config
#include "_config.h"



//////
//
// Classes
//

/** @brief Collection of templated math constants. */
template <class flt_type>
struct APISPEC Constants
{
};
/** @brief Single precision specialization of @ref #Constants . */
template<>
struct APISPEC Constants<float>
{
	static const float pi;
	static const float eps;
	static const float inf;
	static const float max;
};

/** @brief Double precision specialization of @ref #Constants . */
template<>
struct APISPEC Constants<double>
{
	static const double pi;
	static const double eps;
	static const double inf;
	static const double max;
};



#endif // ifndef __UTILS_H__
