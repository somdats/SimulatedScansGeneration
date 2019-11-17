
//////
//
// Includes
//

// C++ STL
#include <iostream>

// Eigen library
#include <Eigen/Geometry>

// Local includes
#include "Ray.h"
#include "Projector.h"
#include "HyperbolicParaboloid.h"
#include "PolyMesh.h"
#include "Scanner.h"
#include "Utils.h"



//////
//
// Local types
//

// Desired floating point precision
typedef double real;

// Eigen type shortcuts
typedef EigenTypes<real>::Vec3 Vec3;
typedef EigenTypes<real>::Vec4 Vec4;
typedef EigenTypes<real>::Mat4 Mat4;



//////
//
// Scanning options
//

#define MODEL_FILEPATH "../../data/Bimba_400k.obj"
#define OUTFILE_BASE   "Z:/staff/SDutta/GlobalRegistration/Sampled/bimba"
#define FOV   60
#define RES_X 128
#define RES_Y 96
#define ROTATION_STEPS_360 12	// 360 / 12 = 30° per step



//////
//
// Helper functions
//

std::string toString (real number)
{
	std::ostringstream strm; strm.precision(3);
	strm << number;
	return std::move(strm.str());
}

std::string toString(int number)
{
	std::ostringstream strm;
	strm << number;
	return std::move(strm.str());
}

std::string toString(unsigned int number)
{
	std::ostringstream strm;
	strm << number;
	return std::move(strm.str());
}



//////
//
// Application entry point
//

int main (int argc, char* argv[])
{
	// Info output...
	std::cout << "Setting up scene...";

	// Prepare model-specific filename base
	const std::string filebase(OUTFILE_BASE);

	// Set up poly mesh object to be scanned
	PolyMesh<real> model(MODEL_FILEPATH);
	//model.genMesh_absValF(1, 1, {-0.75, -0.75}, {0.75, 0.75}, 11);

	// Set up projector
	Projector<real> projector(RES_X, RES_Y);
	projector.applyLookAt(
		// Projector position and orientation, currently at 37.5 units along the world
		// y-axis, looking at the origin (i.e. along positive world y-axis), with up
		// direction aligned with world z-axis
		Vec4(0, 0, 1.25, 1), Vec4(0, 0, 0, 1), Vec3::UnitY()
	);
	projector.applyFovNearFar(FOV, real(0.1), 100);
	projector.writeProjectionMatrix_txt(filebase+"_ProjMat_fov"+toString(FOV) + ".txt");
	std::cout << " done!" << std::endl << std::endl;


	////
	// Phase 1: 360° run

	for (unsigned step=0; step<ROTATION_STEPS_360; step++)
	{
		// Calculate current angle
		real lerp = real(step) / real(ROTATION_STEPS_360),
		     angle = 0 * (1-lerp)  +  Constants<real>::pi*2 * lerp;
		std::string degrees = toString(angle/Constants<real>::pi * 180);

		// Ground truth transformation for current step
		Mat4 GTT =
			// translation (x,y,z), currently none
			EigenHelper<real>::translate44(0, 0, 0) *
			// rotation (axis-angle), currently (step * 30°) around local z-axis
			EigenHelper<real>::rotate44(Vec3::UnitY(), angle);

		// Transform to current scan position
		model.setWorldTransform(GTT.inverse());

		// Scan in current position
		std::cout << "Scanning view " << toString(step+1) << " (" <<degrees<< " deg)...";
		Scanner<real>::SampleArray samples;
		Scanner<real>::scanSurface(&samples, model, projector);
		std::cout << " done!" << std::endl;

		// Write sampling to *.pct and *.ply files
		std::cout << "Writing point cloud files..." << std::endl << "- .pct";
		Scanner<real>::writeSamples_pct(filebase + "_" + degrees + ".pct", samples);
		std::cout << " done!" << std::endl << "- .ply";
		Scanner<real>::writeSamples_ply(filebase + "_" + degrees + ".ply", samples);
		std::cout << " done!" << std::endl;

		// Write resulting camera pose matrix and ground truth model
		std::cout << "Writing ground truth data..." << std::endl;
		std::cout << "- cameraPose";
		projector.writeCameraPose_txt(
			filebase + "_" + degrees + "_cameraPoseMat.txt", model, &GTT
		); std::cout << " done!" << std::endl;
		// Obj looks nice but takes too long to write...
		/*std::cout << "- .obj";
		model.writeTransformedGroundTruthModel_obj(
			filebase + "_" + degrees + "_groundTruthModel.obj", GTT
		); std::cout << " done!" << std::endl;*/
		std::cout << "- .ply";
		/*model.writeTransformedGroundTruthCloud_ply(
			filebase + "_" + degrees + "_groundTruthCloud.ply", GTT
		);*/ std::cout << " done!" << std::endl;

		// For better console readability...
		std::cout << std::endl;
	}


	////
	// Phase 2: top / bottom views

	for (unsigned step=0; step<=1; step++)
	{
		// Calculate current angle
		real lerp = real(step) / 1,
		     angle = -Constants<real>::pi/2 * (1-lerp)  +  Constants<real>::pi/2 * lerp;
		std::string view = step == 0 ? "top" : "bottom";

		// Ground truth transformation for current step
		Mat4 GTT =
			// translation (x,y,z), currently none
			EigenHelper<real>::translate44(0, 0, 0) *
			// rotation (axis-angle), currently (90° - step*180°) around local x-axis
			EigenHelper<real>::rotate44(Vec3::UnitX(), angle);

		// Transform to current scan position
		model.setWorldTransform(GTT.inverse());

		// Scan in current position
		std::cout << "Scanning view " << toString(step+1) << " (" << view << " deg)...";
		Scanner<real>::SampleArray samples;
		Scanner<real>::scanSurface(&samples, model, projector);
		std::cout << " done!" << std::endl;

		// Write sampling to *.pct and *.ply files
		std::cout << "Writing point cloud files..." << std::endl << "- .pct";
		Scanner<real>::writeSamples_pct(filebase + "_" + view + ".pct", samples);
		std::cout << " done!" << std::endl << "- .ply";
		Scanner<real>::writeSamples_ply(filebase + "_" + view + ".ply", samples);
		std::cout << " done!" << std::endl;

		// Write resulting camera pose matrix and ground truth model
		std::cout << "Writing ground truth data..." << std::endl;
		std::cout << "- cameraPose";
		projector.writeCameraPose_txt(
			filebase + "_" + view + "_cameraPoseMat.txt", model, &GTT
		); std::cout << " done!" << std::endl;
		// Obj looks nice but takes too long to write...
		/*std::cout << "- .obj";
		model.writeTransformedGroundTruthModel_obj(
			filebase + "_" + view + "_groundTruthModel.obj", GTT
		); std::cout << " done!" << std::endl;*/
		std::cout << "- .ply";
		model.writeTransformedGroundTruthCloud_ply(
			filebase + "_" + view + "_groundTruthCloud.ply", GTT
		); std::cout << " done!" << std::endl;

		// For better console readability...
		std::cout << std::endl;
	}

	// Done!
	return 0;
}
