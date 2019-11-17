
#ifndef __SCANNER_H__
#define __SCANNER_H__


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

/** @brief Struct representing a scanned sample. */
template <class flt_type>
struct APISPEC ScannedSample
{
	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;


	////
	// Data members

	/** @brief Viewport-space 2D coordinates of the sample. */
	unsigned u, v;

	/** @brief 3D point at this sample. */
	Vec3 point;

	/** @brief Surface normal at this sample. */
	Vec3 normal;
};



//////
//
// Classes
//

/**
 * @brief
 *		Class encapsulating the scanning process by combining a projector and a scannable
 *		surface.
 */
template <class flt_type>
class APISPEC Scanner
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D Homogenous vector type. */
	typedef typename EigenTypes<real>::Vec4 Vec4;

	/** @brief 3D Homogenous transformation matrix type. */
	typedef typename EigenTypes<real>::Mat4 Mat4;

	/** @brief Collection of samples resulting from a scan. */
	typedef std::vector<ScannedSample<real> > SampleArray;


	////
	// Methods

	/**
	 * @brief
	 *		Casts a ray at the given sample position, using multiple threads if desired.
	 */
	static void scanSurface (SampleArray *out, const IScannableObject<real> &object,
	                         const Projector<real> &projector, bool multithread=true);

	/** @brief Writes the given samples to a *.pct 3d-scanner output file. */
	static void writeSamples_pct (const std::string &filename,
	                              const SampleArray &samples);

	/**
	 * @brief
	 *		Writes the given samples to a *.ply file as a point cloud, i.e. without any
	 *		connectivity information.
	 */
	static void writeSamples_ply (const std::string &filename,
	                              const SampleArray &samples);
};



#endif // ifndef __SCANNER_H__
