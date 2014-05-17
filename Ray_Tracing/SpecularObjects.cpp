#include "SpecularObjects.h"

CollisionObject NO_COLLISION;

//START: SpecularPlane declaration;
SpecularPlane::SpecularPlane()
{
	normal << 0,0,0;
	point  << 0,0,0;
	colorIntensity << 0,0,0;
	ObjectCounter++;
}

SpecularPlane::SpecularPlane(Eigen::Vector3f aPoint,Eigen::Vector3f aNormal, Eigen::Vector3f anIntensity)
{ 
	normal = aNormal;
	normal.normalize();
	point = aPoint;
	colorIntensity = anIntensity;
	ObjectCounter++;
}

SpecularPlane::SpecularPlane(float xP,float yP,float zP, float xN,float yN, float zN, float Bi, float Gi, float Ri)
{
	normal << xN,yN,zN;
	normal.normalize();
	point << xP,yP,zP;
	colorIntensity << Bi,Gi,Ri;
	ObjectCounter++;
}

void SpecularPlane::SetCenter(float x,float y,float z)
{
	point << x,y,z;
}

void SpecularPlane::SetCenter(Eigen::Vector3f aCenter)
{
	point = aCenter;
}

Eigen::Vector3f SpecularPlane::GetCenter()
{
	return point;
}

void SpecularPlane::Print()
{
	std::cout << "Plane defined by Point and Normal, Point: " << point[0] <<" , " <<point[1] <<" , " << point[2] << std::endl;
	std::cout << "Plane equation: "<< normal[0]<<"x + "<< normal[1] <<"y+ "<<normal[2]<<"z = "<<normal.dot(point) << std::endl;
	std::cout << "Plane colorIntensity (BGR): " << colorIntensity[0] <<" , " << colorIntensity[1] <<" , " << colorIntensity[2] << std::endl;
	std::cout << "Number of Objects: " << ObjectCounter <<std::endl;
}

void SpecularPlane::SetColorIntensity(float Bi,float Gi,float Ri)
{
	colorIntensity << Bi,Gi,Ri;
}

void SpecularPlane::SetColorIntensity(Eigen::Vector3f aColorIntensity)
{
	colorIntensity = aColorIntensity;
}

Eigen::Vector3f SpecularPlane::GetColorIntensity()
{
	return colorIntensity;
}

bool SpecularPlane::InGeometry(float x,float y, float z)
{
	float dist = normal.dot(point-Eigen::Vector3f(x,y,z));
	//if(dist == 0 || (dist < DIX_FLT_EPS && dist > -DIX_FLT_EPS))
	if(EqCompare(dist,0))
		return true;
	return false;

}

bool SpecularPlane::InGeometry(Eigen::Vector3f aPoint)
{
	float dist = normal.dot(point-aPoint);
	//if(dist == 0 || (dist < DIX_FLT_EPS && dist > -DIX_FLT_EPS))
	if(EqCompare(dist,0))
		return true;
	return false;
}

CollisionObject SpecularPlane::Collision(std::shared_ptr<IRay> aRay)
{
	int flag = ONE_POINT_INSTERSECTION;
	float num,den,d;
	den = normal.dot(aRay->GetDirection());
	if (EqCompare(den,0))
		flag = PARALLEL_LINE;

	num = normal.dot(point-aRay->GetPoint());
	if (EqCompare(num,0) && flag == PARALLEL_LINE)
		flag = LINE_IN_PLANE;
	
	if(flag == LINE_IN_PLANE)
		return CollisionObject(point);
	//Collision occurs in only a point
	else if (flag != PARALLEL_LINE)
	{
		d = num/den;
		return ((d > 0) ? CollisionObject(d*aRay->GetDirection() + aRay->GetPoint()) :NO_COLLISION) ;
	}
	//PARALLEL_LINE
	return NO_COLLISION;
}

vector<shared_ptr<IRay>> SpecularPlane::Reflection(shared_ptr<IRay> aRay, Scene* aCene)
{

	if(aRay->GetReflectionCounter() <= 0)
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	

	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	Eigen::Vector3f ref_dir;
	ref_dir = aRay->GetDirection() - 2*(normal.dot(aRay->GetDirection()))*normal;
	//TODO Add attenuation due to reflection angle;
	std::shared_ptr<IRay> aTempRay(new Ray());
	 vector<shared_ptr<IRay>> ReflectedRays; ReflectedRays.push_back(aTempRay); //ALERT!
	ReflectedRays.at(0)->SetDirection(ref_dir);
	ReflectedRays.at(0)->SetIntensity(aRay->GetIntensity());
	ReflectedRays.at(0)->SetReflectionlimite(aRay->GetReflectionCounter()-1);

#ifdef DELTA
	ReflectedRays.at(0)->SetPoint(a.GetCollisionPosition() + DELTA*ref_dir);
#else
	ReflectedRays.at(0)->SetPoint(a.GetCollisionPosition());
#endif //DELTA != 0

	return ReflectedRays;
}
//END: SpecularPlane declaration;

//START: SpecularSphere declaration;
SpecularSphere::SpecularSphere()
{
	center << 0,0,0;
	radius = 0;
	sqrd_radius = 0;
	colorIntensity <<0,0,0;
	ObjectCounter++;
}

SpecularSphere::SpecularSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity)
{
	center = aCenter;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity = anIntensity;
	ObjectCounter++;
}

SpecularSphere::SpecularSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri)
{
	center << xC,yC,zC;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity << Bi,Gi,Ri;
	ObjectCounter++;
}

void SpecularSphere::SetCenter(float xC,float yC,float zC)
{
	center <<xC,yC,zC;
}

void SpecularSphere::SetCenter(Eigen::Vector3f aCenter)
{
	center = aCenter;
}

void SpecularSphere::Print()
{
	std::cout << "Sphere with radius: "<<radius <<"\nSphere center: "<< center[0] <<" ," << center[1] <<" ," << center[2] <<std::endl;
	std::cout << "Sphere defined by equation: (x "<<-center[0] << ")^2 + (y " << -center[1] << ")^2 + (z " <<-center[2] << ")^2 = " << radius<<"^2\n";
	std::cout << "Sphere colorIntensity (BGR): " << colorIntensity[0] <<" , " << colorIntensity[1] <<" , " << colorIntensity[2] << std::endl;
	std::cout << "Number of Objects: " << ObjectCounter <<std::endl;
}

Eigen::Vector3f SpecularSphere::GetCenter()
{
	return center;
}

void SpecularSphere::SetColorIntensity(float Bi,float Gi,float Ri)
{
	colorIntensity << Bi,Gi,Ri;
}

void SpecularSphere::SetColorIntensity(Eigen::Vector3f aColorIntensity)
{
	colorIntensity = aColorIntensity;
}

Eigen::Vector3f SpecularSphere::GetColorIntensity()
{
	return colorIntensity;
}

bool SpecularSphere::InGeometry(float x,float y, float z)
{
	Eigen::Vector3f val(x,y,z);
	val -= center;
	if(EqCompare(val.norm(),radius))
		return true;
	return false;
}

bool SpecularSphere::InGeometry(Eigen::Vector3f aPoint)
{
	Eigen::Vector3f val = aPoint;
	val -= center;
	if(EqCompare(val.norm(),radius))
		return true;
	return false;
}

CollisionObject SpecularSphere::Collision(shared_ptr<IRay> aRay )
{
	float sqr;
	float d1,d2;
	Eigen::Vector3f dif = aRay->GetPoint()-center;
	float t1,t2,t4;
	
	t4 = aRay->GetDirection().dot(dif);
	t1 = t4*t4;
	t2 = dif.dot(dif);
	sqr = t1 - t2 + sqrd_radius;

	if(sqr < 0) //No collision
		return NO_COLLISION;
	
	d1 = -t4+sqrt(sqr);
	d2 = -t4-sqrt(sqr);
	
	if(d1< 0 && d2 <0) //2 distances are negatives
		return NO_COLLISION;

	if(EqCompare(d1,d2)) //Collision in exactly one point;
		return CollisionObject(aRay->GetPoint()+d1*aRay->GetDirection().normalized());
	
	//Collision in 2 point, so we'll return the one that is nearest to the aRay->GetPoint();
	if(d1 >= 0 && d2 >= 0) //2 distances are positives
	{
		d1 = min(d1,d2);
		return CollisionObject(aRay->GetPoint()+d1*aRay->GetDirection().normalized());
	}
	else if(d1 < 0) // only d1 < 0
	{
		return CollisionObject(aRay->GetPoint()+d2*aRay->GetDirection().normalized());
	}
	//only d2 < 0
	return CollisionObject(aRay->GetPoint()+d1*aRay->GetDirection().normalized());

}

vector<shared_ptr<IRay>> SpecularSphere::Reflection(shared_ptr<IRay> aRay, Scene* aCene)
{
	if(aRay->GetReflectionCounter() <= 0)
		return vector<shared_ptr<IRay>>();

	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	Eigen::Vector3f ref_dir;
	ref_dir = aRay->GetDirection() - 2*(Normal(a.GetCollisionPosition()).dot(aRay->GetDirection()))*Normal(a.GetCollisionPosition());

	std::shared_ptr<IRay> aTempRay(new Ray());
	 vector<shared_ptr<IRay>> ReflectedRays; ReflectedRays.push_back(aTempRay); //ALERT!
	ReflectedRays.at(0)->SetDirection(ref_dir);
	ReflectedRays.at(0)->SetIntensity(aRay->GetIntensity());
	ReflectedRays.at(0)->SetReflectionlimite(aRay->GetReflectionCounter()-1);
#ifdef DELTA
	ReflectedRays.at(0)->SetPoint(a.GetCollisionPosition() + DELTA*ref_dir);
#else
	ReflectedRays.at(0)->SetPoint(a.GetCollisionPosition());
#endif //DELTA != 0
	return ReflectedRays;
}

Eigen::Vector3f SpecularSphere::Normal(Eigen::Vector3f aSurfacePoint)
{
	return (aSurfacePoint-center).normalized();
}