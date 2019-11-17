
//////
//
// Includes
//

// C++ STL
#include <vector>
#include <fstream>
#include <utility>

// Local includes
#include "Ray.h"
#include "Projector.h"
#include "IScannableObject.h"

// Implemented header
#include "Scanner.h"



//////
//
// Class implementation
//

// Scanner
//

template <class flt_type>
void Scanner<flt_type>::scanSurface(
	SampleArray *out, const IScannableObject<real> &object,
	const Projector<real> &projector, bool multithread
)
{
	Projector<real>::Res2 res = std::move(projector.getResolution());
	const unsigned num_pixels = res.x() * res.y();

	// Scanning loop
	out->clear(); out->reserve(num_pixels);
	Mat4 viewMatrix(std::move(projector.getViewMatrix())),
	     viewMatrixIT = std::move(viewMatrix.inverse().transpose());
	if (multithread)
	{
		#pragma omp parallel for schedule(guided)
		for (int pixel=0; pixel<int(num_pixels); pixel++)
		{
			// Initialize sample with current pixel coordinates
			ScannedSample<real> sample;
			sample.u = pixel % res.x();
			sample.v = pixel / res.x();

			// Cast sampling ray from current pixel
			Ray<real> ray = projector.castRay(real(sample.u), real(sample.v));

			// Intersect with surface
			Intesection<real> isect = object.intersectRay(ray);
			if (isect.hit)
			{
				/*sample.point  = std::move(isect.pos);
				sample.normal = std::move(isect.normal);*/

				// Sanity check
				if (std::isnan(isect.pos.x()) || std::isnan(isect.pos.y()) ||
				    std::isnan(isect.pos.z()))
					throw std::runtime_error("[Scanner] !!!INTERNAL ERROR!!!");
				if (std::isinf(isect.pos.x()) || std::isinf(isect.pos.y()) ||
				    std::isinf(isect.pos.z()))
					throw std::runtime_error("[Scanner] !!!INTERNAL ERROR!!!");

				// Apply camera space transformation to scanned point
				Vec4 v_h(isect.pos.x(), isect.pos.y(), isect.pos.z(), 1);
				v_h = viewMatrix * v_h;
				sample.point.col(0) << v_h.x()/v_h.w(), v_h.y()/v_h.w(), v_h.z()/v_h.w();

				// Apply camera space transformation to normal at scanned point
				v_h.col(0) << isect.normal.x(), isect.normal.y(), isect.normal.z(), real(0);
				v_h = viewMatrixIT * v_h;
				sample.normal.col(0) << v_h.x(), v_h.y(), v_h.z();

				#pragma omp critical (store_sample) 
				{
					// Thread-safe sample store
					out->push_back(sample);
				}
			}
		}

		// Sort samples vector
		std::sort(out->begin(), out->end(), [] (const ScannedSample<real> &elem1,
		                                        const ScannedSample<real> &elem2) -> bool
		{
			return elem1.v <= elem2.v && elem1.u < elem2.u;
		});
	}
	else
	{
		for (unsigned pixel=0; pixel<num_pixels; pixel++)
		{
			// Initialize sample with current pixel coordinates
			ScannedSample<real> sample;
			sample.u = pixel % res.x();
			sample.v = pixel / res.x();

			// Cast sampling ray from current pixel
			Ray<real> ray = projector.castRay(real(sample.u), real(sample.v));

			// Intersect with surface
			Intesection<real> isect = object.intersectRay(ray);
			if (isect.hit)
			{
				/*sample.point  = std::move(isect.pos);
				sample.normal = std::move(isect.normal);*/

				// Sanity check
				if (std::isnan(isect.pos.x()) || std::isnan(isect.pos.y()) ||
				    std::isnan(isect.pos.z()))
					throw std::runtime_error("[Scanner] !!!INTERNAL ERROR!!!");
				if (std::isinf(isect.pos.x()) || std::isinf(isect.pos.y()) ||
				    std::isinf(isect.pos.z()))
					throw std::runtime_error("[Scanner] !!!INTERNAL ERROR!!!");

				// Apply camera space transformation to scanned point
				Vec4 v_h(isect.pos.x(), isect.pos.y(), isect.pos.z(), 1);
				v_h = viewMatrix * v_h;
				sample.point.col(0) << v_h.x()/v_h.w(), v_h.y()/v_h.w(), v_h.z()/v_h.w();

				// Apply camera space transformation to normal at scanned point
				v_h.col(0) << isect.normal.x(), isect.normal.y(), isect.normal.z(), real(0);
				v_h = viewMatrixIT * v_h;
				sample.normal.col(0) << v_h.x(), v_h.y(), v_h.z();

				// Store sample
				out->push_back(sample);
			}
		}
	}

	// Finish up
	out->shrink_to_fit();
}

template <class flt_type>
void Scanner<flt_type>::writeSamples_pct(const std::string &filename,
                                         const SampleArray &samples)
{
	std::fstream pctfile(filename, std::fstream::out);

	// *.pct header
	pctfile << "u\tv\tz\tx\ty\tGreyValue" << std::endl;

	// *.pct specific floating point settings
	pctfile.setf(std::ios::fixed); pctfile.precision(4);

	// Write samples
	for (unsigned i = 0; i<samples.size(); i++)
	{
		pctfile
			<< samples[i].u << "\t" << samples[i].v << "\t"
			<< samples[i].point.z() << "\t" << samples[i].point.x() << "\t"
			<< samples[i].point.y() << "\t100" << std::endl;
	}
}

template <class flt_type>
void Scanner<flt_type>::writeSamples_ply(const std::string &filename,
                                         const SampleArray &samples)
{
	std::fstream plyfile(filename, std::fstream::out);

	// *.ply header
	plyfile << "ply" << std::endl
	        << "format ascii 1.0" << std::endl
	        << "element vertex " << samples.size() << std::endl
	        << "property float x" << std::endl
	        << "property float y" << std::endl
	        << "property float z" << std::endl
	        << "property float nx" << std::endl
	        << "property float ny" << std::endl
	        << "property float nz" << std::endl
	        << "end_header" << std::endl;

	// *.ply specific floating point settings
	plyfile.setf(std::ios::fixed); plyfile.precision(6);

	// Write samples
	for (unsigned i = 0; i<samples.size(); i++)
	{
		plyfile
			<< samples[i].point.x() << " " << samples[i].point.y() << " "
			<< samples[i].point.z() << " "
			<< samples[i].normal.x() << " " << samples[i].normal.y() << " "
			<< samples[i].normal.z() << std::endl;
	}
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC Scanner<float>;
template APISPEC Scanner<double>;
