
//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <fstream>
#include <stdexcept>

// Local includes
#include "Ray.h"
#include "Utils.h"

// Implemented header
#include "Projector.h"



//////
//
// Class implementation
//

// Projector
//

template <class flt_type>
Projector<flt_type>::Projector()
	: resolution(128, 96), SAR(1)
{
	viewMatrix.setIdentity();
	viewMatrixInv.setIdentity();
	projectionMatrix.setIdentity();
	projectionMatrixInv.setIdentity();
}

template <class flt_type>
Projector<flt_type>::Projector(unsigned resX, unsigned resY, real SAR)
	: resolution(resX, resY), SAR(SAR)
{
	viewMatrix.setIdentity();
	viewMatrixInv.setIdentity();
	projectionMatrix.setIdentity();
	projectionMatrixInv.setIdentity();
}

template <class flt_type>
Projector<flt_type>::Projector(const Res2 &resolution, real SAR)
	: resolution(resolution), SAR(SAR)
{
	viewMatrix.setIdentity();
	viewMatrixInv.setIdentity();
	projectionMatrix.setIdentity();
	projectionMatrixInv.setIdentity();
}

template <class flt_type>
Projector<flt_type>::~Projector() {}

template <class flt_type>
Ray<flt_type> Projector<flt_type>::castRay (real posX, real posY) const
{
	// Undo projector screen transformation
	Vec4
		near_h(
			(posX / (resolution.x()/real(2))) - real(1),
			real(1) - (posY / (resolution.y()/real(2))),
			-1, 1
		),
		far_h(near_h.x(), near_h.y(), 1, 1);

	// Unproject to view space
	near_h = projectionMatrixInv * near_h;
	near_h /= near_h.w();
	far_h = projectionMatrixInv * far_h;
	far_h /= far_h.w();
	// Transform into world space direction
	near_h = viewMatrixInv * near_h;
	near_h /= near_h.w();
	far_h  = viewMatrixInv * far_h;
	far_h /= far_h.w();

	// Store resulting ray
	Ray<real> result;
	result.direction.x() = far_h.x() - near_h.x();
	result.direction.y() = far_h.y() - near_h.y();
	result.direction.z() = far_h.z() - near_h.z();
	result.direction.normalize();
	result.origin = std::move(near_h);
	return std::move(result);
}

template <class flt_type>
void Projector<flt_type>::applyLookAt (const Vec4 &pos, const Vec4 &target,
                                       const Vec3 &up)
{
	// Sanity checks
	if (pos == target)
		throw std::move(
			std::runtime_error("[Projector] Specified position and focus point must not"
			                   " be identical!")
		);
	if (up.isZero())
		throw std::move(
			std::runtime_error("[Projector] Specified up direction must not be the null"
			                   " vector!")
		);

	// Determine view coordinate system x and z basis vectors
	Vec3 eye   (   pos.x()/pos.w(),       pos.y()/pos.w(),       pos.z()/pos.w()),
	     origin(target.x()/target.w(), target.y()/target.w(), target.z()/target.w()),
	     zAxis (   eye.x()-origin.x(),    eye.y()-origin.y(),    eye.z()-origin.z());
	zAxis.normalize();
	Vec3 xAxis(std::move(zAxis.cross(std::move(up.normalized()))));

	// Check for collinear view / up directions
	if (xAxis.isZero())
		throw std::move(
			std::runtime_error("[Projector] Specified up vector must not be collinear"
			                   " with viewing direction!")
		);

	// Determine view coordinate system y basis vector
	xAxis.normalize();
	Vec3 yAxis(std::move(xAxis.cross(zAxis)));

	// Apply matrix coefficients
	viewMatrix(3, 2) = viewMatrix(3, 1) = viewMatrix(3, 0) = 0;
	viewMatrix(3, 3) = 1;
	viewMatrix(0, 0) = xAxis.x();
	viewMatrix(0, 1) = xAxis.y();
	viewMatrix(0, 2) = xAxis.z();
	viewMatrix(1, 0) = yAxis.x();
	viewMatrix(1, 1) = yAxis.y();
	viewMatrix(1, 2) = yAxis.z();
	viewMatrix(2, 0) = zAxis.x();
	viewMatrix(2, 1) = zAxis.y();
	viewMatrix(2, 2) = zAxis.z();
	viewMatrix(0, 3) = (-xAxis).dot(eye);
	viewMatrix(1, 3) = (-yAxis).dot(eye);
	viewMatrix(2, 3) = (-zAxis).dot(eye);

	// Calculate inverse
	viewMatrixInv = std::move(viewMatrix.inverse());
}

template <class flt_type>
void Projector<flt_type>::applyFovNearFar (real FoV, real near, real far)
{
	// Sanity checks
	if (near <= real(0))
		throw std::move(
			std::runtime_error("[Projector] Specified near projection plane must be"
			                   " greater than zero!")
		);
	if (far <= near)
		throw std::move(
			std::runtime_error("[Projector] Specified far projection plane must be"
			                   " behind near projection plane!")
		);

	// Determine intrinsic parameters
	real FoV_half = (FoV/real(2)) * Constants<real>::pi / real(180),
	     f = std::cos(FoV_half) / std::sin(FoV_half),
	     aspect = resolution.x() * SAR / resolution.y();

	// Apply matrix coefficients
	projectionMatrix.setZero();
	projectionMatrix(0, 0) = f / aspect;
	projectionMatrix(1, 1) = f;
	projectionMatrix(2, 2) = (far+near) / (near-far);
	projectionMatrix(3, 2) = -1;
	projectionMatrix(2, 3) = (real(2)*far*near) / (near-far);

	// Calculate inverse
	projectionMatrixInv = std::move(projectionMatrix.inverse());
}

template <class flt_type>
void Projector<flt_type>::writeCameraPose_txt(
	const std::string &filename, const IScannableObject<real> &object, Mat4 *cameraPose
) const
{
	// Calculate camera pose matrix
	Mat4 CPM = viewMatrix * object.getWorldTransform();

	// Write to file
	std::fstream txtfile(filename, std::fstream::out);
	txtfile << CPM.data()[0];
	for (unsigned i=1; i<16; i++)
		if ((i%4)+1 < 4) // Skip bottom row, it's implied to be (0 0 0 1)
			txtfile << " " << CPM.data()[i];
	txtfile << std::endl;

	// Report camera pose matrix if requested
	if (cameraPose)
		*cameraPose = std::move(CPM);
}

template <class flt_type>
void Projector<flt_type>::writeProjectionMatrix_txt (const std::string &filename) const
{
	std::fstream txtfile(filename, std::fstream::out);

	for (unsigned i=0; i<16; i++)
		txtfile << projectionMatrix.data()[i] << " ";
	txtfile << std::endl;
}

template <class flt_type>
void Projector<flt_type>::setViewMatrix (const Mat4 &mat)
{
	bool invertible;
	mat.computeInverseWithCheck(viewMatrixInv, invertible);
	if (invertible)
		viewMatrix = mat;
	else
		throw std::move(
			std::runtime_error("[Projector] Specified view matrix is not invertible!")
		);
}
template <class flt_type>
typename EigenTypes<flt_type>::Mat4 Projector<flt_type>::getViewMatrix (void) const
{
	return viewMatrix;
}
template <class flt_type>
void Projector<flt_type>::getViewMatrix (Mat4 *out) const
{
	*out = viewMatrix;
}

template <class flt_type>
void Projector<flt_type>::setProjectionMatrix (const Mat4 &mat)
{
	bool invertible;
	mat.computeInverseWithCheck(projectionMatrixInv, invertible);
	if (invertible)
		projectionMatrix = mat;
	else
		throw std::move(std::runtime_error(
			"[Projector] Specified projection matrix is not invertible!"
		));
}
template <class flt_type>
typename EigenTypes<flt_type>::Mat4 Projector<flt_type>::getProjectionMatrix (void) const
{
	return projectionMatrix;
}
template <class flt_type>
void Projector<flt_type>::getProjectionMatrix (Mat4 *out) const
{
	*out = projectionMatrix;
}

template <class flt_type>
void Projector<flt_type>::setResolution (const Res2 &res)
{
	resolution = res;
}
template <class flt_type>
typename EigenTypes<unsigned>::Vec2 Projector<flt_type>::getResolution (void) const
{
	return resolution;
}
template <class flt_type>
void Projector<flt_type>::getResolution (Res2 *out) const
{
	*out = resolution;
}

template <class flt_type>
void Projector<flt_type>::setSAR(real value)
{
	SAR = value;
}
template <class flt_type>
flt_type Projector<flt_type>::getSAR(void) const
{
	return SAR;
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC Projector<float>;
template APISPEC Projector<double>;
