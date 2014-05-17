
#ifndef SPECULAROBJECTS_H
#define SPECULAROBJECTS_H

#include "primitives.h"
#include <Eigen/Geometry>
#include <float.h>
#include <iostream>
#include "Functions.h"
#include <memory>
#include "defines.h"

/* 
	Specular Objects are objects that only have mirror like reflection.
	The reflected ray don't get any information of the object color and the reflected direction is given by Snell's Law
	
	Moreover, these objects where used as the "root" for all the other types of planes an spheres, and because of that
	they have functions to set and get color intensity even when they don't need.

	Finally, all of them inherit from IPhysicalObject.
*/


/*
	SpecularPlane:
	Defines a plane that only has specular reflection
	Even though it has a color intensity, this isn't used in the current implementation.
	All planes are defined by a normal vector and a point.

*/
class SpecularPlane : virtual public IPhysicalObject
{
protected:
	Eigen::Vector3f normal;	//Vector that stores the plane normal
	Eigen::Vector3f point;  //Vector that stores the plane definition point
public:
	//Defines a plane with all his variables set to zero
	SpecularPlane();
	//Defines a plane passing by aPoint, and with a given aNormal vector. Also takes anIntensity in BGR order.
	SpecularPlane(Eigen::Vector3f aPoint,Eigen::Vector3f aNormal, Eigen::Vector3f anIntensity);
	//Defines a plane passing by a point (xP,yP,zP), and with a given normal vector (xN,yN,zN). Also takes anIntensity (Bi,Gi,Ri) in BGR order.
	SpecularPlane(float xP,float yP,float zP, float xN,float yN, float zN, float Bi, float Gi, float Ri);

	//heritage

	//Sets the plane definition point to a new position (x,y,z)
	virtual void SetCenter(float x,float y,float z);
	//Sets the plane definition point to a new position given by aCenter
	virtual void SetCenter(Eigen::Vector3f aCenter);
	//Returns the plane definition point
	virtual Eigen::Vector3f GetCenter();

	//A function that prints the object information
	virtual void Print();

	//Color control

	//Sets and the color intensity of the plane using 3 floats in BGR order
	virtual void SetColorIntensity(float Bi,float Gi,float Ri);
	//Sets and the color intensity of the plane using a Eigen::Vector3f in BGR order
	virtual void SetColorIntensity(Eigen::Vector3f aColorIntensity);
	//Returns the color intensity of the object in a Eigen::Vector3f
	virtual Eigen::Vector3f GetColorIntensity();

	//Geometry control
	
	//Checks if point(x,y,z) in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(float x,float y, float z);
	//Checks if aPoint in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(Eigen::Vector3f aPoint);


	//Check if a IRay collides with this object
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay );

	//Reflection control
	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occur,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//As this object is specular id doesn't need information about the scene that he is into.
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL);

	virtual ~SpecularPlane() {}
};

/*
	SpecularSphere:
	Defines a sphere that only has specular reflection
	Even though it has a color intensity, this isn't used in the current implementation.
	All Spheres are defined by a center and a radius.
*/

class SpecularSphere :virtual public IPhysicalObject
{
protected:
	Eigen::Vector3f center;		//Container for the sphere center
	float radius,sqrd_radius;	//Containers for the sphere radius and the squared radius (to decrease some repetitive computations)
public:
	//Defines a sphere with all his variables set to zero
	SpecularSphere();
	//Defines a sphere with aCenter and aRadius. Also takes an anIntensity in BGR order.
	SpecularSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity);
	//Defines a sphere with a center(xC,yC,zC) and aRadius. Also takes anInentisty(Bi,Gi,Ri) in BGR order.
	SpecularSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri);

	//Sets the sphere center to a new position (xC,yC,zC).
	virtual void SetCenter(float xC,float yC,float zC);
	//Sets the sphere center to a new position aCenter.
	virtual void SetCenter(Eigen::Vector3f aCenter);
	//Returns the sphere center.
	virtual Eigen::Vector3f GetCenter();

	//A function that prints the object information
	virtual void Print();

	//Color control
	//Sets and the color intensity of the object using 3 floats in BGR order
	virtual void SetColorIntensity(float Bi,float Gi,float Ri);
	//Sets and the color intensity of the object using a Eigen::Vector3f in BGR order
	virtual void SetColorIntensity(Eigen::Vector3f aColorIntensity);
	//Returns the color intensity of the object in a Eigen::Vector3f
	virtual Eigen::Vector3f GetColorIntensity();

	//Geometry control
	
	//Checks if point(x,y,z) in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(float x,float y, float z);
	//Checks if aPoint in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(Eigen::Vector3f aPoint);


	//Check if a IRay collides with this object
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay );

	//Reflection control
	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occour,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//As this object is specular id doesn't need information about the scene that he is into.
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL);
	
	//Returns what is the normal in a point in the Surface of sphere. Always, the normal point to outside the sphere.
	//Warning: It's assumed that the point is in the surface of the sphere.
	virtual Eigen::Vector3f Normal(Eigen::Vector3f aSurfacePoint);

	virtual ~SpecularSphere() {}
};


#endif //!SPECULAROBJECTS_H