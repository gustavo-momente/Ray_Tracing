#ifndef MULTIPLEREFLECTIONOBJECTS_H
#define MULTIPLEREFLECTIONOBJECTS_H

#include "DiffuseObjects.h"
#include "SpecularObjects.h"
#include "primitives.h"
#include "defines.h"

/*
	MultiReflectionObjects are objects that can have reflective, diffuse and refractive output rays.
	For reflective and diffuse they use the same model as SpecularObjects and DiffuseObjects, respectively.
	The refraction is given by Snell's law. So the existence of refraction implies the existence of reflection.

	Moreover, when instancing an object we can have the following combinations:
		* Specular-Diffuse-Refractive(SDR) object
			> The refractive_index is used as the object refractive index (it must be >=1)
			> The lambertian_contribution is used as an multiplier to the outgoing color intensity of the diffuse reflection
			> The specular_contribution is unused
			> The rafractive_contribution is unused
			> The contribution of refraction and reflexion in the outgoing color intensity is obtained from Snell's law.

		* Specular-Diffuse(SD) object
			> The refractive_index must be set to something less then 1 or or SDR object will be defined
			> The lambertian_contribution is used as an multiplier to the outgoing color intensity of the diffuse reflection
			> The specular_contribution is used as an multiplier to the outgoing color intensity of the specular reflection
			> The rafractive_contribution is unused
			> In real world lambertian_contribution + specular_contribution <= 1, but a blind eye is turn to this as the user
				can play with those indices. 

		* Specular-Refractive(SR) object
			> The refractive_index is used as the object refractive index
			> The lambertian_contribution must be zero or a SDR object will be defined
			> The specular_contribution is unused
			> The rafractive_contribution is unused
			> The contribution of refraction and reflexion in the outgoing color intensity is obtained from Snell's law.

		* Diffuse(D) object
			> The refractive_index must be set to something less then 1 or SDR object will be defined
			> The lambertian_contribution is used as an multiplier to the outgoing color intensity of the diffuse reflection
			> The specular_contribution must be set to zero or a SR object will be defined
			> The rafractive_contribution is unused.


		* Specular(S) object
			> The refractive_index must be set to something less then 1 or SR object will be defined
			> The lambertian_contribution must be Set to zero or a SD object will be defined
			> The specular_contribution is used as an multiplier to the outgoing color intensity of the specular reflection
			> The rafractive_contribution is unused

		The rafractive_contribution is unused in all current cases, but he's left, because some future implementations can use it
		to give the use more power in the intensity control

		Finally, a new interface IMRObject is defined and all MultiReflectionObjects should inherit from it 
*/

class IMRObject : virtual public IPhysicalObject
{
protected:
	float refractive_index; 
	bool refraction_flag;	//Tells the object that there will be refraction or no 
	float lambertian_contribution; 
	float specular_contribution;
	float refractive_contribution;
	bool specular_flag, diffuse_flag;
	Eigen::Vector3f _normal; //_normal = -normal


public:
	//Updates the refractive index of the object to the new value aIndex, also updates the flag relative to refraction.
	virtual void setRefractiveIndex(float aIndex) {refractive_index = aIndex; refraction_flag = (aIndex <= 1) ? false:true ; }
	//Returns the refractive index of the object
	virtual float getRefractiveIndex() { return refractive_index; }
	
	//Updates the all the contributions to new values and their respective flags
	//virtual void setContributions(float aLambertian, float aSpecular,float aRefractive) { lambertian_contribution = aLambertian; specular_contribution = aSpecular; refractive_contribution = aRefractive; CheckContributions();}
	
	//Returns all the tree contributions in a Eigen::Vector3f, the order is the following one : lambertian,specular,refractive.
	virtual Eigen::Vector3f GetContributions() { return Eigen::Vector3f(lambertian_contribution,specular_contribution,refractive_contribution);}

	//Returns what will be the specular contribution when refraction occurs
	//aCi is the cosine of the incident ray with the normal.
	//aCt is the cosine of the refracted ray with the _normal
	//Please, refer to http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
	//for more information.
	virtual float GetReflectance(float aCi, float aCt, float n2,float n1 = N1);

	//virtual void CheckContributions();
	
	//Updates the 3 flags
	virtual void UpdateFlags();
	
	virtual ~IMRObject() {}
};

/*
	MRPlane:
	Defines a plane that can have all kinds of light comportment.
	All planes are defined by a normal vector and a point.
	Even thought, a Refractive plane doesn't have sense physically it's implemented to follow
	the Interface and object construction.
*/

class MRPlane : virtual public IMRObject, virtual public SpecularPlane
{
public:
	//Defines a plane with all his variables set to zero
	MRPlane();
	//Defines a plane passing by aPoint, and with a given aNormal vector. Also takes anIntensity in BGR order.
	//For the extensive comportment referring the indices and contributions, please, consult the documentation or the "MultiReflectionObjects.h" file.
	MRPlane(Eigen::Vector3f aPoint, Eigen::Vector3f aNormal, Eigen::Vector3f aColorIntensity, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive);
	//Defines a plane passing by a point (xP,yP,zP), and with a given normal vector (xN,yN,zN). Also takes anIntensity (Bi,Gi,Ri) in BGR order.
	//For the extensive comportment referring the indices and contributions, please, consult the documentation or the "MultiReflectionObjects.h" file.
	MRPlane(float aP_x, float aP_y, float aP_z, float aN_x, float aN_y, float aN_z, float aBi, float aGi, float aRi, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive);

	//Sets the plane definition point to a new position (x,y,z)
	virtual void SetCenter(float x,float y,float z) { SpecularPlane::SetCenter(x,y,z); }
	//Sets the plane definition point to a new position given by aCenter
	virtual void SetCenter(Eigen::Vector3f aCenter) { SpecularPlane::SetCenter(aCenter); }
	//Returns the plane definition point
	virtual Eigen::Vector3f GetCenter() { return SpecularPlane::GetCenter(); }

	//A function that prints the object information
	virtual void Print() { SpecularPlane::Print(); }

	//Color control

	//Sets and the color intensity of the plane using 3 floats in BGR order
	virtual void SetColorIntensity(float B,float G,float R) { SpecularPlane::SetColorIntensity(B,G,R); }
	//Sets and the color intensity of the plane using a Eigen::Vector3f in BGR order
	virtual void SetColorIntensity(Eigen::Vector3f aColorIntensisy) { SpecularPlane::SetColorIntensity(aColorIntensisy); }
	//Returns the color intensity of the object in a Eigen::Vector3f	
	virtual Eigen::Vector3f GetColorIntensity() {return SpecularPlane::GetColorIntensity(); }

	//Geometry control

	//Checks if point(x,y,z) in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(float x,float y, float z) { return SpecularPlane::InGeometry(x,y,z); }
	//Checks if aPoint in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(Eigen::Vector3f aPoint) { return SpecularPlane::InGeometry(aPoint); }


	//Check if a IRay collides with this object
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay ) { return SpecularPlane::Collision(aRay); }
	
	//Check if the object is refractive returns true if it's else returns false
	virtual bool IsRefractive() {return refraction_flag;}

	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occur,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//This object needs information about the scene he's into.
	//WARNING:  if aScene = NULL an empty vector will be returned.	
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL);

	virtual ~MRPlane() {}
};

/*
	MRSphere:
	Defines a sphere that can have all kinds of light comportment.
	All Spheres are defined by a center and a radius
*/
class MRSphere : virtual public IMRObject, virtual public SpecularSphere
{
public:
	//Defines a sphere with all his variables set to zero 
	MRSphere();
	//Defines a sphere with aCenter, aRadius and a color defined by anIntensity in BGR order.
	//For the extensive comportment referring the indices and contributions, please, consult the documentation or the "MultiReflectionObjects.h" file.
	MRSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive);
	//Defines a sphere with center(xC,yV,zC), aRadius and a color defined by (Bi,Gi,Ri) in BGR order.
	//For the extensive comportment referring the indices and contributions, please, consult the documentation or the "MultiReflectionObjects.h" file.
	MRSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive);

	//Sets the sphere center to a new position (xC,yC,zC).
	virtual void SetCenter(float x,float y,float z) { SpecularSphere::SetCenter(x,y,z); }
	//Sets the sphere center to a new position aCenter.
	virtual void SetCenter(Eigen::Vector3f aCenter) { SpecularSphere::SetCenter(aCenter); }
	//Returns the sphere center.
	virtual Eigen::Vector3f GetCenter() { return SpecularSphere::GetCenter(); }

	//A function that prints the object information
	virtual void Print() { SpecularSphere::Print(); } 

	//Color control
	
	//Sets and the color intensity of the object using 3 floats in BGR order
	virtual void SetColorIntensity(float B,float G,float R) { SpecularSphere::SetColorIntensity(B,G,R); }
	//Sets and the color intensity of the object using a Eigen::Vector3f in BGR order
	virtual void SetColorIntensity(Eigen::Vector3f aColorIntensisy) { SpecularSphere::SetColorIntensity(aColorIntensisy); }
	//Returns the color intensity of the object in a Eigen::Vector3f
	virtual Eigen::Vector3f GetColorIntensity() {return SpecularSphere::GetColorIntensity(); }

	//Geometry control

	//Checks if point(x,y,z) in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(float x,float y, float z) { return SpecularSphere::InGeometry(x,y,z); }
	//Checks if aPoint in the space belongs to the surface of the object
	//Returns true if belongs, else false
	virtual bool InGeometry(Eigen::Vector3f aPoint) { return SpecularSphere::InGeometry(aPoint); }

	//Check if a IRay collides with this object
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay ) { return SpecularSphere::Collision(aRay); }
	
	//Reflection control

	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occur,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//This object needs information about the scene he's into.
	//WARNING:  if aScene = NULL an empty vector will be returned.
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL);

	virtual bool IsRefractive() {return refraction_flag;}

	virtual ~MRSphere() {}
};


#endif // !MULTIPLEREFLECTIONOBJECTS_H
