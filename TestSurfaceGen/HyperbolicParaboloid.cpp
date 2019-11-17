
//////
//
// Includes
//

// C++ STL
#include <cmath>

// Local includes
#include "Ray.h"

// Implemented header
#include "HyperbolicParaboloid.h"



//////
//
// Class implementation
//

// HyperbolicParaboloid
//

template <class flt_type>
HyperbolicParaboloid<flt_type>::HyperbolicParaboloid(real cx, real cy, real sx, real sy,
                                                     real x0, real y0, real xmin,
                                                     real ymin, real xmax, real ymax)
	: coeffs(cx, cy), scale(sx, sy), offset(x0, y0), rect_min(xmin, ymin),
	  rect_max(xmax, ymax)
{
	worldTransform.setIdentity();
	worldTransformInv.setIdentity();
}

template <class flt_type>
HyperbolicParaboloid<flt_type>::~HyperbolicParaboloid() {}

template <class flt_type>
bool HyperbolicParaboloid<flt_type>::intersectRay(real *t, const Ray<real> &ray) const
{
	// Local helper
	struct _
	{
		inline static bool isInRect(const Vec3 &point, const Vec2 &rmin, const Vec2 &rmax)
		{
			return
				point.x() >= rmin.x() && point.x() <= rmax.x() &&
				point.y() >= rmin.y() && point.y() <= rmax.y();
		}
	};
	// Transform ray to local coordinate system
	Vec4 rayPoint(std::move(ray.point4At(1)));
	rayPoint = worldTransformInv * rayPoint;
	Ray<real> rLocal;
	rLocal.origin = worldTransformInv * ray.origin;
	rLocal.direction.x() =
		(rayPoint.x()/rayPoint.w()) - (rLocal.origin.x()/rLocal.origin.w());
	rLocal.direction.y() =
		(rayPoint.y()/rayPoint.w()) - (rLocal.origin.y()/rLocal.origin.w());
	rLocal.direction.z() =
		(rayPoint.z()/rayPoint.w()) - (rLocal.origin.z()/rLocal.origin.w());
	rLocal.direction.normalize();

	// ToDo: Replace in expressions below
	real cx = coeffs.x(), cy = coeffs.y(), sx = scale.x(), sy = scale.y(), x0 = offset.x(), y0 = offset.y(),
	     px = rLocal.origin.x(), py = rLocal.origin.y(), pz = rLocal.origin.z(),
	     dx = rLocal.direction.x(), dy = rLocal.direction.y(), dz = rLocal.direction.z();

	// Ray-surface intersection solved for t as a quadratic
	real
		tmp1 = (dz - real(2)*cx*dx*px*sx*sx - real(2)*cy*dy*py*sy*sy + real(2)*cx*dx*sx*x0 + real(2)*cy*dy*sy*y0),
		t1 = (real(1) / (real(2)*(-cx*dx*dx*sx*sx - cy*dy*dy*sy*sy)))*(-dz + real(2)*cx*dx*px*sx*sx + real(2)*cy*dy*py*sy*sy - real(2)*cx*dx*sx*x0 - real(2)*cy*dy*sy*y0 + std::sqrt(tmp1*tmp1 - real(4)*(-cx*dx*dx*sx*sx - cy*dy*dy*sy*sy)*(pz - cx*px*px*sx*sx - cy*py*py*sy*sy + real(2)*cx*px*sx*x0 - cx*x0*x0 + real(2)*cy*py*sy*y0 - cy*y0*y0))),
		tmp2 = (dz - real(2)*cx*dx*px*sx*sx - real(2)*cy*dy*py*sy*sy + real(2)*cx*dx*sx*x0 + real(2)*cy*dy*sy*y0),
		t2 = (real(1) / (real(2)*(-cx*dx*dx*sx*sx - cy*dy*dy*sy*sy)))*(-dz + real(2)*cx*dx*px*sx*sx + real(2)*cy*dy*py*sy*sy - real(2)*cx*dx*sx*x0 - real(2)*cy*dy*sy*y0 - std::sqrt(tmp2*tmp2 - real(4)*(-cx*dx*dx*sx*sx - cy*dy*dy*sy*sy)*(pz - cx*px*px*sx*sx - cy*py*py*sy*sy + real(2)*cx*px*sx*x0 - cx*x0*x0 + real(2)*cy*py*sy*y0 - cy*y0*y0)));

	// Check for valid t
	if (std::isnan(t2) || t2 < real(0))
	{
		if (std::isnan(t1) || t1 < real(0))
			// Report non-intersection
			return false;

		// Check if intersection is within boundary
		Vec3 isect = ray.point3At(t1);
		if (_::isInRect(isect, rect_min, rect_max))
		{
			*t = t1;
			return true;
		}
		return false;
	}
	if (std::isnan(t1) || t1 < real(0))
	{

		// Check if intersection is within boundary
		Vec3 isect = ray.point3At(t2);
		if (_::isInRect(isect, rect_min, rect_max))
		{
			*t = t2;
			return true;
		}
		return false;
	}

	// Check for intersection inside boundary region
	Vec3 isect1 = ray.point3At(t1), isect2 = ray.point3At(t2);
	if (!_::isInRect(isect2, rect_min, rect_max))
	{
		if (!_::isInRect(isect1, rect_min, rect_max))
			// Report non-intersection
			return false;

		// Report t1
		*t = t1;
		return true;
	}
	if (!_::isInRect(isect1, rect_min, rect_max))
	{
		// Report t2
		*t = t2;
		return true;
	}

	// Report closest t
	*t = t1 < t2 ? t1 : t2;
	return true;
}

template <class flt_type>
void HyperbolicParaboloid<flt_type>::setWorldTransform(const Mat4 &mat)
{
	bool invertible;
	mat.computeInverseWithCheck(worldTransformInv, invertible);
	if (invertible)
		worldTransform = mat;
	else
		throw std::move(
			std::runtime_error("[HyperbolicParaboloid] Specified world coordinate system"
			                   " transformation matrix is not invertible!")
		);
}
template <class flt_type>
typename EigenTypes<flt_type>::Mat4 HyperbolicParaboloid<flt_type>::
	getWorldTransform(void) const
{
	return worldTransform;
}
template <class flt_type>
void HyperbolicParaboloid<flt_type>::getWorldTransform(Mat4 *out) const
{
	*out = worldTransform;
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC HyperbolicParaboloid<float>;
template APISPEC HyperbolicParaboloid<double>;
