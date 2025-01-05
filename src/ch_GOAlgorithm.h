#ifndef CH_GOALGORITHM_H
#define CH_GOALGORITHM_H
#pragma comment(lib, "gsl.lib")
#pragma comment(lib, "cblas.lib")

// the gsl library for computing matrix inverse
#include "gsl/gsl_linalg.h"


// collision checker
#include "ch_segmentTriangleCollisionChecker.h"

// CHAI3D includes
#include "chai3d.h"

using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------



class ch_GOAlgorithm
{
public:

	// constructor 
	ch_GOAlgorithm() {};

	// destructor 
	virtual ~ch_GOAlgorithm() {};

	// the feedback forces according to the GO algorithm are computed here
	cVector3d ch_GOComputeForces(ch_segmentTriangleCollisionChecker* collision_checker, cVector3d& next_proxy_pos, const cVector3d& current_device_pos);

	// fill out the 6x6 matrix for GO position computation here
	void ch_fillGOPositionOptimisation(ch_segmentTriangleCollisionChecker* collision_checker, const cVector3d& current_device_pos, cVector3d& next_proxy_pos);

	// compute feedback force according to the Hooke's law
	void ch_computeStiffForce(const cVector3d& next_proxy_pos, const cVector3d& current_device_pos);
	
	
protected:
	
	// the computed feedback force according to the Hooke's law
	cVector3d return_force;
};

#endif