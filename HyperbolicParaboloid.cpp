
//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <limits>

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
	worldTransformInvT.setIdentity();
}

template <class flt_type>
HyperbolicParaboloid<flt_type>::~HyperbolicParaboloid() {}

template <class flt_type>
Intesection<flt_type> HyperbolicParaboloid<flt_type>::intersectRay(
	const Ray<real> &ray
) const
{
	// Local helpers
	auto isInRect = [] (const Vec4 &point, const Vec2 &rmin, const Vec2 &rmax) -> bool
	{
		return
			point.x() >= rmin.x() && point.x() <= rmax.x() &&
			point.y() >= rmin.y() && point.y() <= rmax.y();
	};
	auto toVec3 = [] (const Vec4 &vec_h) -> Vec3
	{
		return
			std::move(Vec3(
				vec_h.x()/vec_h.w(), vec_h.y()/vec_h.w(), vec_h.z()/vec_h.w()
			));
	};

	// Transform ray to local coordinate system
	Vec4 rayPoint(std::move(ray.point4At(1)));
	rayPoint = worldTransformInv * rayPoint;
	Ray<real> rLocal;
	rLocal.origin = worldTransformInv * ray.origin;
	rLocal.origin /= rLocal.origin.w();
	rLocal.direction.x() =
		(rayPoint.x() / rayPoint.w()) - rLocal.origin.x();
	rLocal.direction.y() =
		(rayPoint.y() / rayPoint.w()) - rLocal.origin.y();
	rLocal.direction.z() =
		(rayPoint.z() / rayPoint.w()) - rLocal.origin.z();
	rLocal.direction.normalize();

	// ToDo: Replace in expressions below
	const real
		cx = coeffs.x(), cy = coeffs.y(), sx = scale.x(), sy = scale.y(),
		x0 = offset.x(), y0 = offset.y(), px = rLocal.origin.x(), py = rLocal.origin.y(),
		pz = rLocal.origin.z(), dx = rLocal.direction.x(), dy = rLocal.direction.y(),
		dz = rLocal.direction.z();
	// ^ related local helpers
	const Mat4 &transIT = worldTransformInvT;
	auto normalAt = [&transIT, cx, cy, sx, sy, x0, y0] (const Vec4 &isect) -> Vec3
	{
		Vec4 grad(
			-(real(2)*cx*(isect.x()-x0))/(sx*sx), -(real(2)*cy*(isect.y()-y0))/(sy*sy),
			1, 0
		);
		grad.normalize();
		grad = transIT * grad;
		return std::move(Vec3(grad.x(), grad.y(), grad.z()));
	};
	auto fEval = [cx, cy, sx, sy, x0, y0] (const Vec4 &query) -> Vec4
	{
		const real _x = sx*(query.x()-x0), _y = sy*(query.y()-y0);
		return
			std::move(Vec4(
				query.x(), query.y(),
				cx/(sx*sx*sx*sx)*_x*_x + cy/(sy*sy*sy*sy)*_y*_y,
				1
			));
	};

	// Ray-surface intersection solved for t as a quadratic
	real
		sx_sqr = sx * sx, sy_sqr = sy * sy, dx_sqr = dx * dx, dy_sqr = dy * dy,
		_cx_dx2__sx2 = (cx*dx_sqr)/sx_sqr, _cy_dy2__sy2 = (cy*dy_sqr)/sy_sqr,
		_2_cx_dx_x0__sx2 = (real(2)*cx*dx*x0)/sx_sqr,
		_2_cy_dy_y0__sy2 = (real(2)*cy*dy*y0)/sy_sqr,
		_2_cx_dx_px__sx2 = (real(2)*cx*dx*px)/sx_sqr,
		_2_cy_dy_py__sy2 = (real(2)*cy*dy*py)/sy_sqr;

	real
		denom = real(2)*(_cx_dx2__sx2 + _cy_dy2__sy2),
		tmp = (
			-dz - _2_cx_dx_x0__sx2 + _2_cx_dx_px__sx2 - _2_cy_dy_y0__sy2 + _2_cy_dy_py__sy2
		);
	real sqr =
		tmp * tmp
		- real(4)*(_cx_dx2__sx2 + _cy_dy2__sy2)*(
			-pz
			+ (   cx  *(x0*x0) )/sx_sqr
			- (real(2)*cx*x0*px)/sx_sqr
			+ (   cx  *(px*px) )/sx_sqr
			+ (   cy  *(y0*y0) )/sy_sqr
			- (real(2)*cy*y0*py)/sy_sqr
			+ (   cy  *(py*py) )/sy_sqr
			);

	// Intersection test
	Intesection<real> isect; isect.hit = false;
	if (sqr < real(0))
		// Ray misses the surface
		return std::move(isect);

	// Special case: denominator zero
	if (std::abs(denom) < (sx+sy)*std::numeric_limits<real>::epsilon())
	{
		// Handle as ray-plane intersection within epsilon

		// Ray-plane intersection solved for t
		Vec3 refplane_normal(0, 0, 1), ray_origin_neg(
			-rLocal.origin.x(), -rLocal.origin.y(), -rLocal.origin.z()
		);
		real nom = ray_origin_neg.dot(refplane_normal);
		denom = rLocal.direction.dot(refplane_normal);

		// Sanity check
		if (denom == real(0))
			throw std::runtime_error("!!!INTERNAL ERROR!!!");

		// Test intersection
		real t_plane = nom / denom;
		Vec4 ispos = rLocal.point4At(t_plane);
		if (isInRect(ispos, rect_min, rect_max))
		{
			// Report plane intersection
			isect.hit = true;
			isect.t = t_plane;
			isect.pos = std::move(toVec3(worldTransform * fEval(ispos)));
			isect.normal = std::move(normalAt(ispos));
		}
		return std::move(isect);
	}

	// Solve quadratic
	real
		t1 = (  dz + _2_cx_dx_x0__sx2 - _2_cx_dx_px__sx2
		      - std::sqrt(sqr) + _2_cy_dy_y0__sy2 - _2_cy_dy_py__sy2) / denom,
		t2 = (  dz + _2_cx_dx_x0__sx2 - _2_cx_dx_px__sx2
		      + std::sqrt(sqr) + _2_cy_dy_y0__sy2 - _2_cy_dy_py__sy2) / denom;

	// Check for valid t
	if (std::isnan(t2) || t2 < real(0))
	{
		if (std::isnan(t1) || t1 < real(0))
			// Report non-intersection
			return std::move(isect);

		// Check if intersection is within boundary
		Vec4 ispos = rLocal.point4At(t1);
		if (isInRect(ispos, rect_min, rect_max))
		{
			isect.hit = true;
			isect.t = t1;
			isect.pos = std::move(toVec3(worldTransform * ispos));
			isect.normal = std::move(normalAt(ispos));
		}
		return std::move(isect);
	}
	if (std::isnan(t1) || t1 < real(0))
	{
		// Check if intersection is within boundary
		Vec4 ispos = rLocal.point4At(t2);
		if (isInRect(ispos, rect_min, rect_max))
		{
			isect.hit = true;
			isect.t = t2;
			isect.pos = std::move(toVec3(worldTransform * ispos));
			isect.normal = std::move(normalAt(ispos));
		}
		return std::move(isect);
	}

	// Check for intersection inside boundary region
	Vec4 isect1 = rLocal.point4At(t1), isect2 = rLocal.point4At(t2);
	if (!isInRect(isect2, rect_min, rect_max))
	{
		if (!isInRect(isect1, rect_min, rect_max))
			// Report non-intersection
			return std::move(isect);

		// Report t1
		isect.hit = true;
		isect.t = t1;
		isect.pos = std::move(toVec3(worldTransform * isect1));
		isect.normal = std::move(normalAt(isect1));
		return std::move(isect);
	}
	if (!isInRect(isect1, rect_min, rect_max))
	{
		// Report t2
		isect.hit = true;
		isect.t = t2;
		isect.pos = std::move(toVec3(worldTransform * isect2));
		isect.normal = std::move(normalAt(isect2));
		return std::move(isect);
	}

	// Report closest t
	isect.hit = true;
	if (t1 <= t2)
	{
		isect.t = t1;
		isect.pos = std::move(toVec3(worldTransform * isect1));
		isect.normal = std::move(normalAt(isect1));
	}
	else
	{
		isect.t = t2;
		isect.pos = std::move(toVec3(worldTransform * isect2));
		isect.normal = std::move(normalAt(isect2));
	}
	return std::move(isect);
}

template <class flt_type>
void HyperbolicParaboloid<flt_type>::setWorldTransform(const Mat4 &mat)
{
	bool invertible;
	mat.computeInverseWithCheck(worldTransformInv, invertible);
	if (invertible)
	{
		worldTransform = mat;
		worldTransformInvT = worldTransformInv.transpose();
	}
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
