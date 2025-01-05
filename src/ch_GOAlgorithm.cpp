#include "ch_GOAlgorithm.h"
#include <vector>


// the feedback forces according to the GO algorithm are computed here and solve for the next best GO position
cVector3d ch_GOAlgorithm::ch_GOComputeForces(ch_segmentTriangleCollisionChecker* collision_checker, cVector3d& next_proxy_pos, const cVector3d& current_device_pos)
{
	// fill out the 6x6 matrix for GO position computation 
	ch_fillGOPositionOptimisation(collision_checker, current_device_pos, next_proxy_pos);

	// compute feedback force according to the Hooke's law
	ch_computeStiffForce(next_proxy_pos, current_device_pos);

	return(return_force);
}

// fill out the 6x6 matrix for GO position computation here
void ch_GOAlgorithm::ch_fillGOPositionOptimisation(ch_segmentTriangleCollisionChecker* collision_checker, const cVector3d& current_device_pos, cVector3d& next_proxy_pos)
{
	gsl_matrix_view m;
	gsl_vector_view b;

	// the solution vector
	gsl_vector *x;

	int s;
	gsl_permutation *p;



	// which constraints are active? - fill out values in the 6x6 matrix - see structure of matrix in the chapter
	// on haptic rendering with the GO algorithm


	// check for redundancy, eg. for the cube, triangles 0 and 1 are contained in the same plane,
	// in which case we will have redundant rows and columns
	// only the case for one redundancy is taken care of below, but we should also worry about
	// cases in which collision_checker->collidedTriangleIndex.size() > 2
	if (collision_checker->collidedTriangleIndex.size() == 2)
	{
		cVector3d plane1, plane2;

		plane1.copyfrom(collision_checker->planesForTriangles[collision_checker->collidedTriangleIndex[0]].ch_getPlaneNormal());
		plane2.copyfrom(collision_checker->planesForTriangles[collision_checker->collidedTriangleIndex[1]].ch_getPlaneNormal());
		
		if (plane1.distance(plane2) < SMALL_NUM)
		{
			collision_checker->ch_popBack();
		}
	}


	// which constraints are active? - fill out values in the 6x6 matrix - see structure of matrix in the chapter
	// on haptic rendering with the GO algorithm
	switch (collision_checker->collidedTriangleIndex.size())
	{
	case 1:
	{
			break;
	}

	case 2:
	{
			break;
	}

	case 3:
	{
			break;
	}

	default:
	{
			   double a_data[] = { 1, 0, 0,
				   0, 1, 0,
				   0, 0, 1 };
			   double b_data[] = { current_device_pos.x(), current_device_pos.y(), current_device_pos.z() };

			   m = gsl_matrix_view_array(a_data, 3, 3);
			   b = gsl_vector_view_array(b_data, 3);

			   x = gsl_vector_alloc(3);
			   p = gsl_permutation_alloc(3);
	}
	}

	s = ((m.matrix.size1 % 2) == 0) ? 1 : -1;

    next_proxy_pos.set(gsl_vector_get(x, 0), gsl_vector_get(x, 1), gsl_vector_get(x, 2));

	gsl_permutation_free(p);
	gsl_vector_free(x);
}



// compute feedback force according to the Hooke's law
void ch_GOAlgorithm::ch_computeStiffForce(const cVector3d& next_proxy_pos, const cVector3d& current_device_pos)
{
	// compute spring resistance force between the new calculated GO location and the goal	
	next_proxy_pos.subr(current_device_pos, return_force);
	
	// stiffness of 40 assumed here
	return_force.mul(40);
}