#include "DiffuseObjects.h"

//START: DiffusePlane Implementation
DiffusePlane::DiffusePlane()
{
	normal << 0,0,0;
	point  << 0,0,0;
	colorIntensity << 0,0,0;
	ObjectCounter++;
}

DiffusePlane::DiffusePlane(Eigen::Vector3f aPoint,Eigen::Vector3f aNormal, Eigen::Vector3f anIntensity)
{ 
	normal = aNormal;
	normal.normalize();
	point = aPoint;
	colorIntensity = anIntensity;
	ObjectCounter++;
}

DiffusePlane::DiffusePlane(float xP,float yP,float zP, float xN,float yN, float zN, float Bi, float Gi, float Ri)
{
	normal << xN,yN,zN;
	normal.normalize();
	point << xP,yP,zP;
	colorIntensity << Bi,Gi,Ri;
	ObjectCounter++;
}

vector<std::shared_ptr<IRay>> DiffusePlane::Reflection(std::shared_ptr<IRay> aRay, Scene* aScene)
{

	if(aRay->GetReflectionCounter() <= 0)
	{
		vector<std::shared_ptr<IRay>> temp;
		return temp;
	}

	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<std::shared_ptr<IRay>> temp;
		return temp;
	}
	
	int flag;
	vector<Ilight*> lights = aScene->Getlight();
	vector<std::shared_ptr<IRay>> light_rays;
	std::vector<Ilight*>::iterator begin = lights.begin();

	for(std::vector<Ilight*>::iterator it = lights.begin(); it != lights.end(); ++it)
	{
		std::shared_ptr<IRay> aTempRay(new Ray(a.GetCollisionPosition(),(*it)->GetCenter()-a.GetCollisionPosition(),aRay->GetIntensity(),aRay->GetReflectionCounter()-1,it-begin));
		light_rays.push_back(aTempRay);

		flag = 0;
		if(aRay->GetDirection().dot(normal) < 0)
		{
			if(light_rays.back()->GetDirection().dot(normal) > 0) flag = 1;
		}
		else
			if(light_rays.back()->GetDirection().dot(normal) < 0) flag = 1;
		if(aScene->Collision(light_rays.back()) || flag == 0 ) //If there isn't a direct path to the light, remove last added ray
		{ 
			light_rays.pop_back();
		}
		else
		{
			float gain = light_rays.back()->GetDirection().dot(normal);
			gain = ( (gain >= 0) ? gain : -gain);
			light_rays.back()->SetIntensity(TWOWAY_MUL*gain*colorIntensity.cwiseProduct( light_rays.back()->GetIntensity() ) );
#ifdef DELTA
			light_rays.back()->SetPoint(light_rays.back()->GetPoint()+light_rays.back()->GetDirection()*DELTA);  
#endif // DELTA != 0

#ifdef TWOWAY
			std::shared_ptr<IRay> aTempRay2(new Ray(light_rays.back()->GetPoint(),light_rays.back()->GetDirection(),light_rays.back()->GetIntensity(),light_rays.back()->GetReflectionCounter(),-1));
			light_rays.push_back(aTempRay2);
#endif //TWOWAY
		}
	}
	return light_rays;
}
//END: DiffusePlane Implementation


//START: DiffuseSphere Implementation
DiffuseSphere::DiffuseSphere()
{
	center << 0,0,0;
	radius = 0;
	sqrd_radius = 0;
	colorIntensity <<0,0,0;
	ObjectCounter++;
}

DiffuseSphere::DiffuseSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity)
{
	center = aCenter;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity = anIntensity;
	ObjectCounter++;
}

DiffuseSphere::DiffuseSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri)
{
	center << xC,yC,zC;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity << Bi,Gi,Ri;
	ObjectCounter++;
}

vector<std::shared_ptr<IRay>> DiffuseSphere::Reflection(std::shared_ptr<IRay> aRay, Scene* aScene)
{
	if(aRay->GetReflectionCounter() <= 0)
	{
		vector<std::shared_ptr<IRay>> temp;
		return temp;
	}

	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<std::shared_ptr<IRay>> temp;
		return temp;
	}
	
	int flag;
	Eigen::Vector3f normal = Normal(a.GetCollisionPosition());
	vector<Ilight*> lights = aScene->Getlight();
	vector<std::shared_ptr<IRay>> light_rays;
	std::vector<Ilight*>::iterator begin = lights.begin();

	for(std::vector<Ilight*>::iterator it = lights.begin(); it != lights.end(); ++it)
	{
		std::shared_ptr<IRay> aTempRay(new Ray(a.GetCollisionPosition(),(*it)->GetCenter()-a.GetCollisionPosition(),aRay->GetIntensity(),aRay->GetReflectionCounter()-1,it-begin));
		light_rays.push_back(aTempRay);

		flag = 0;
		if(aRay->GetDirection().dot(normal) < 0)
		{
			if(light_rays.back()->GetDirection().dot(normal) > 0) flag = 1;
		}
		//uncommented the two next lines
		else
			if(light_rays.back()->GetDirection().dot(normal) < 0) flag = 1;

		if(aScene->Collision(light_rays.back()) || flag == 0) //If there isn't a direct path to the light, remove last added ray
		{ 
			light_rays.pop_back();
		}
		else
		{
			float gain = light_rays.back()->GetDirection().normalized().dot(normal);
			gain = ( (gain >= 0) ? gain : 0);
			light_rays.back()->SetIntensity( TWOWAY_MUL*gain*colorIntensity.cwiseProduct( light_rays.back()->GetIntensity() ) );
#ifdef DELTA
			light_rays.back()->SetPoint(light_rays.back()->GetPoint()+light_rays.back()->GetDirection()*DELTA);
#endif // DELTA != 0

#ifdef TWOWAY
			std::shared_ptr<IRay> aTempRay2(new Ray(light_rays.back()->GetPoint(),light_rays.back()->GetDirection(),light_rays.back()->GetIntensity(),light_rays.back()->GetReflectionCounter(),-1));
			light_rays.push_back(aTempRay2);
#endif //TWOWAY

		}
	}
	return light_rays;
}
//END: DiffuseSphere Implementation



