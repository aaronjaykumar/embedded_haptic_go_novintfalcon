#include "ch_segmentTriangleCollisionChecker.h"


// constructor
ch_segmentTriangleCollisionChecker::ch_segmentTriangleCollisionChecker(cMultiMesh* obj)
{
	// the virtual object that we will work with
	object = obj;
	unsigned int multi_mesh_idx = 0;
	
	for (unsigned int i = 0; i < object->getNumTriangles(); i++)
	{
		
		ch_plane temp_plane;
		planesForTriangles.push_back(temp_plane);

		planesForTriangles[i].ch_computePlane(i, object); //ch_plane.cpp
		
	}
}


// check for GO-device segment-triangle collisions
void ch_segmentTriangleCollisionChecker::ch_checkCollisions(const cVector3d& lastDevicePosition, const cVector3d& currentDevicePosition, cVector3d intersectionPoint)
{
	unsigned int i;

	for (i = 0; i < object->getNumTriangles(); i++)
	{
		int collided = -3;

		
		//cVector3d intersectionPoint;	// intersection between segment and triangle, if it has taken place
		collided = ch_checkSegTriangleCollision(i, lastDevicePosition, currentDevicePosition, intersectionPoint);
	
		if (collided == 1)
		{
			collidedTriangleIndex.push_back(i);
		}
	}
}


// called from ch_checkCollisions()
int ch_segmentTriangleCollisionChecker::ch_checkSegTriangleCollision(const unsigned int TriangleIndex, const cVector3d& lastDevicePosition, const cVector3d& currentDevicePosition, cVector3d& intersectionPoint)
{
	cVector3d v0, v1, v2;	// vertices of the triangle
	cVector3d v01, v02;		// vectors from vertex 0 to vertices 1 and 2	
	cVector3d ray_direction;	// direction of the segment
	cVector3d plane_normal;
	double d, t;

	//for (unsigned int j = 0; j < object->getNumTriangles(); j++){
		
		v0 = object->getVertexPos(object->getMesh(0)->m_triangles->getVertexIndex0(TriangleIndex));
		v1 = object->getVertexPos(object->getMesh(0)->m_triangles->getVertexIndex1(TriangleIndex));
		v2 = object->getVertexPos(object->getMesh(0)->m_triangles->getVertexIndex2(TriangleIndex));
		

		v0 = cAdd(object->getMesh(0)->getGlobalPos(), cMul(object->getMesh(0)->getGlobalRot(), v0));
		v1 = cAdd(object->getMesh(0)->getGlobalPos(), cMul(object->getMesh(0)->getGlobalRot(), v1));
		v2 = cAdd(object->getMesh(0)->getGlobalPos(), cMul(object->getMesh(0)->getGlobalRot(), v2));

		v1.subr(v0, v01);
		v2.subr(v0, v02);

		// compute the plane which contains the triangle: plane represented as ax+by+cz+d = 0
		// first compute the plane normal
		v02.crossr(v01, plane_normal);	// don't know if this normal points inwards or outwards from the object
		plane_normal.normalize();	// gives a, b and c coeffs. of the plane

		// compute coeff. d 
		d = -cDot(plane_normal, v0);

		currentDevicePosition.subr(lastDevicePosition, ray_direction);

		if (cDot(plane_normal, ray_direction) < SMALL_NUM)
			return 0;	// no intersection because plane and ray (almost) parallel
		else
			t = -(cDot(plane_normal, lastDevicePosition) + d) / (cDot(plane_normal, ray_direction));

		// check if t corresonds to a point on the segment or to one outside ie
		if (t < 0 || t > 1)
			return -1;	// no intersection because point outside segment
		else
		{
			// check if the intersection point lies inside the triangle		
			lastDevicePosition.addr((cMul(t, ray_direction)), intersectionPoint);

			if (ch_pointInTriangle(intersectionPoint, v0, v1, v2))
				return 1;	// intersection! - common point found to lie on the segment as well as inside the triangle
			else
				return -2;	// no intersection because point lies outside triangle				
		}
}



// check if a given point lies inside a given triangle
bool ch_segmentTriangleCollisionChecker::ch_pointInTriangle(const cVector3d& intersectionPoint, const cVector3d& vertex0, const cVector3d& vertex1, const cVector3d& vertex2)
{
	// check for all three sides of the triangle	
	if (ch_sameSide(intersectionPoint, vertex2, vertex0, vertex1)	\
		&& ch_sameSide(intersectionPoint, vertex0, vertex1, vertex2)	\
		&& ch_sameSide(intersectionPoint, vertex1, vertex2, vertex0))
		return true;
	else
		return false;
}



bool ch_segmentTriangleCollisionChecker::ch_sameSide(const cVector3d& intersectionPoint, const cVector3d& v3, const cVector3d& v1, const cVector3d& v2)
{
	return false;
}



// highlight the collided triangles
void ch_segmentTriangleCollisionChecker::ch_highlightTriangles()
{
	vector<int>::iterator it;
	it = collidedTriangleIndex.begin();

	if (collidedTriangleIndex.size() > 0)
	{
		while (it != collidedTriangleIndex.end())
		{
						
			int vertex0 = object->getMesh(0)->m_triangles->getVertexIndex0(*it);
			int vertex1 = object->getMesh(0)->m_triangles->getVertexIndex1(*it);
			int vertex2 = object->getMesh(0)->m_triangles->getVertexIndex2(*it);

			
			object->getMesh(0)->m_triangles->m_vertices->setColor(vertex0, 1.0, 0.0, 0.0);
			object->getMesh(0)->m_triangles->m_vertices->setColor(vertex1, 1.0, 0.0, 0.0);
			object->getMesh(0)->m_triangles->m_vertices->setColor(vertex2, 1.0, 0.0, 0.0);

			++it;
		}
		collidedTriangleIndex.clear();
	}
}

// 'un'-highlight the collided triangles after some time
void ch_segmentTriangleCollisionChecker::ch_unHighlightTriangles()
{
	// reset colors 
	for (unsigned int i = 0; i < object->getNumVertices(); i++)
	{
		cColorb color;
		color.set(
			GLuint(0xff * (1.0 + object->getMesh(0)->m_vertices->getLocalPos(i).x()) / (2.0 * 1.0)),
			GLuint(0xff * (1.0 + object->getMesh(0)->m_vertices->getLocalPos(i).y()) / (2.0 * 1.0)),
			GLuint(0xff * object->getMesh(0)->m_vertices->getLocalPos(i).z() / 2 * 1.0)
			);
		object->getMesh(0)->m_vertices->setColor(i, color);
		
	}

}