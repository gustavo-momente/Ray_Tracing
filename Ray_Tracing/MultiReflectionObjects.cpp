
#include "MultiReflectionObjects.h"

//void IMRObject::CheckContributions()
//{
//	float sum = lambertian_contribution + specular_contribution + refractive_contribution;
//	while(sum != 1)
//	{
//		std::cout <<"Sum of contributions must be equals 1\nLambertian : "<<lambertian_contribution <<" , Specular: "<< specular_contribution << " , Refractive : ";
//		std::cout << refractive_contribution << "\nSum: "<< sum <<"Please enter new ones (comma separated) : "; 
//#ifndef DEBUG				
//		std::scanf("%f,%f,%f", &lambertian_contribution,&specular_contribution,&refractive_contribution);
//#endif //!DEBUG
//		sum = lambertian_contribution + specular_contribution + refractive_contribution;
//	}
//}

void IMRObject::UpdateFlags()
{
	specular_flag = (specular_contribution > 0 || refractive_index >= 1) ? true:false; 
	diffuse_flag  = (lambertian_contribution > 0) ? true:false;
	//refractive_flag = (refractive_contribution > 0) ? true:false;
	refraction_flag = (refractive_index >= 1) ? true:false;
}

float IMRObject::GetReflectance(float aCi, float aCt,float n2,float n1)
{
	float Rt = (n1*aCi - n2*aCt)/(n1*aCi + n2*aCt); 
	Rt *=Rt;
	float Rp = (n2*aCi - n1*aCt)/(n2*aCi + n1*aCt); 
	Rp *=Rp;

	return (Rp+Rt)/2;
}

MRPlane::MRPlane()
{
	normal << 0,0,0;
	_normal << 0,0,0;
	point  << 0,0,0;
	colorIntensity << 0,0,0;
	refractive_index = 1;
	refraction_flag = false;	 
	lambertian_contribution = 0; 
	specular_contribution = 0;
	refractive_contribution = 0;
	UpdateFlags();
}

MRPlane::MRPlane(Eigen::Vector3f aPoint, Eigen::Vector3f aNormal, Eigen::Vector3f aColorIntensity, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive) 
{
	normal  = aNormal;
	normal.normalize();
	_normal = -normal;
	point  = aPoint;
	colorIntensity = aColorIntensity;
	refractive_index = aRefractiveIndex;
	lambertian_contribution = aLamabertian; 
	specular_contribution = aSpecular;
	refractive_contribution = aRefractive;
	//CheckContributions();
	UpdateFlags();
} 

MRPlane::MRPlane(float aP_x, float aP_y, float aP_z, float aN_x, float aN_y, float aN_z, float aBi, float aGi, float aRi, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive)
{
	normal  << aN_x,aN_y,aN_z;
	normal.normalize();
	_normal = -normal;
	point  << aP_x,aP_y,aP_z;
	colorIntensity << aBi,aGi,aRi;
	refractive_index = aRefractiveIndex;
	lambertian_contribution = aLamabertian; 
	specular_contribution = aSpecular;
	refractive_contribution = aRefractive;
	//CheckContributions();
	UpdateFlags();
}


vector<std::shared_ptr<IRay>> MRPlane::Reflection(std::shared_ptr<IRay> aRay, Scene* aScene)
{

	if(aRay->GetReflectionCounter() <= 0 || aScene == NULL) //Test if ReflectionCounter <= 0 or aScene == NULL
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	//Check if the Ray effectually hits the object
	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	vector<shared_ptr<IRay>> ReflectedRays; 

	bool side_flag = (normal.dot(aRay->GetDirection()) < 0) ? true : false; //true : normal is pointing against incoming ray, so will use normal, else we use _normal;
	
	Eigen::Vector3f normalU = (side_flag) ? normal:_normal; 


	float Ci  = -normalU.dot(aRay->GetDirection()); //Cos(Theta_i)
	//Ci = (Ci > 0) ? Ci : -Ci;
	float St2 = (N1/refractive_index)*(N1/refractive_index)*(1-Ci*Ci);
	bool total_internal = (St2 > 1) ? true : false;
	
	float Ct = (!total_internal) ? sqrt(1-St2) : 0;
	float S_contr = (refraction_flag) ? GetReflectance(Ci,Ct,refractive_index) : specular_contribution;
	float R_contr = 1 - S_contr;

	/* Specular Contribution
	*/
	if(specular_flag)
	{
		std::shared_ptr<IRay> aTempRay(new Ray());
		ReflectedRays.push_back(aTempRay);

		Eigen::Vector3f ref_dir;

		ref_dir = aRay->GetDirection() - 2*(normalU.dot(aRay->GetDirection()))*normalU;

		ReflectedRays.back()->SetDirection(ref_dir);
		ReflectedRays.back()->SetIntensity(aRay->GetIntensity()*S_contr);
		ReflectedRays.back()->SetReflectionlimite(aRay->GetReflectionCounter()-1);

	#ifdef DELTA
		ReflectedRays.back()->SetPoint(a.GetCollisionPosition() + DELTA*ref_dir);
	#else
		ReflectedRays.back()->SetPoint(a.GetCollisionPosition());
	#endif //DELTA != 0
	}

	/* Diffuse Contribution
	*/
	if(diffuse_flag)
	{
		int flag = 1;
		float gain;
		vector<Ilight*> lights = aScene->Getlight();
		std::vector<Ilight*>::iterator begin = lights.begin();
		for(std::vector<Ilight*>::iterator it = lights.begin(); it != lights.end(); ++it)
		{
			std::shared_ptr<IRay> aTempRayD(new Ray(a.GetCollisionPosition(),(*it)->GetCenter()-a.GetCollisionPosition(),aRay->GetIntensity(),aRay->GetReflectionCounter()-1,it-begin));
			ReflectedRays.push_back(aTempRayD);
			
			//Checking if the light is in the same side as the origin ray, only if non-refractive
			
			if(!refraction_flag)
			{
				flag = 0;
				if(aRay->GetDirection().dot(normal) < 0)
				{
					if(ReflectedRays.back()->GetDirection().dot(normal) > 0) flag = 1;
				}
				else
					if(ReflectedRays.back()->GetDirection().dot(normal) < 0) flag = 1;
			}

			if(aScene->Collision(ReflectedRays.back()) || flag == 0 ) //If there isn't a direct path to the light, remove last added ray
			{ 
				ReflectedRays.pop_back();
			}
			else
			{
				gain = ReflectedRays.back()->GetDirection().dot(normalU);
				gain = ( (gain >= 0) ? gain : 0);
				ReflectedRays.back()->SetIntensity( TWOWAY_MUL*lambertian_contribution*gain*colorIntensity.cwiseProduct( ReflectedRays.back()->GetIntensity() ) );
				#ifdef DELTA
					ReflectedRays.back()->SetPoint(ReflectedRays.back()->GetPoint()+ReflectedRays.back()->GetDirection()*DELTA);  
				#endif // DELTA != 0
				
				#ifdef TWOWAY
					std::shared_ptr<IRay> aTempRay2(new Ray(ReflectedRays.back()->GetPoint(),ReflectedRays.back()->GetDirection(),ReflectedRays.back()->GetIntensity(),ReflectedRays.back()->GetReflectionCounter(),-1));
					ReflectedRays.push_back(aTempRay2);
				#endif //TWOWAY

			}
		}
	}

	/* Refractive Contribution
	*/
	if(refraction_flag && !total_internal)
	{

		float r = refractive_index*refractive_index - N1*N1;
		float c = -normalU.dot(aRay->GetDirection());
		Eigen::Vector3f refrac_dir;

		refrac_dir = aRay->GetDirection()  + c*normalU - std::sqrt( r + c*c)*normalU;

		#ifdef DELTA
			std::shared_ptr<IRay> aTempRayr(new Ray(a.GetCollisionPosition() + refrac_dir*DELTA,refrac_dir,aRay->GetIntensity()*R_contr,aRay->GetReflectionCounter()-1));
		#else
			std::shared_ptr<IRay> aTempRayr(new Ray(a.GetCollisionPosition(),refrac_dir,aRay->GetIntensity()*R_contr,aRay->GetReflectionCounter()-1));
		#endif //DELTA
		
		ReflectedRays.push_back(aTempRayr);
	}

	return ReflectedRays;
}


/*Begin MRSphere
*/
MRSphere::MRSphere()
{
	center << 0,0,0;
	radius = 0;
	sqrd_radius = 0;
	colorIntensity <<0,0,0;

	refractive_index = 0; 	 
	lambertian_contribution = 0; 
	specular_contribution = 0;
	refractive_contribution = 0;
	UpdateFlags();
}

MRSphere::MRSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive)
{
	center = aCenter;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity = anIntensity;
	refractive_index = aRefractiveIndex;
	//refraction_flag = (refractive_index >= 1) ? true:false;	 
	lambertian_contribution = aLamabertian; 
	specular_contribution = aSpecular;
	refractive_contribution = aRefractive;
	//CheckContributions();
	UpdateFlags();
}

MRSphere::MRSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri, float aRefractiveIndex, float aLamabertian, float aSpecular, float aRefractive)
{
	center << xC,yC,zC;
	radius = aRadius;
	sqrd_radius = radius*radius;
	colorIntensity << Bi,Gi,Ri;
	refractive_index = aRefractiveIndex;
	//refraction_flag = (refractive_index >= 1) ? true:false;	 
	lambertian_contribution = aLamabertian; 
	specular_contribution = aSpecular;
	refractive_contribution = aRefractive;
	//CheckContributions();
	UpdateFlags();
}

vector<std::shared_ptr<IRay>> MRSphere::Reflection(std::shared_ptr<IRay> aRay, Scene* aScene)
{

	if(aRay->GetReflectionCounter() <= 0 || aScene == NULL) //Test if ReflectionCounter <= 0 or aScene == NULL
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	//Check if the Ray effectually hits the object
	CollisionObject a;
	a = Collision(aRay);
	if(!a.IsThereCollision())
	{
		vector<shared_ptr<IRay>> temp;
		return temp;
	}
	
	vector<shared_ptr<IRay>> ReflectedRays; 
	Eigen::Vector3f normal = Normal(a.GetCollisionPosition());
	_normal = -normal;

	bool side_flag = (normal.dot(aRay->GetDirection()) < 0) ? true : false; //true : normal is pointing against incoming ray, so will use normal, else we use _normal;
	
	float refractive_index_i = (side_flag) ? N1:refractive_index;
	float refractive_index_e = (side_flag) ? refractive_index:N1;

	Eigen::Vector3f normalU = (side_flag) ? normal:_normal; 

	float Ci  = -normalU.dot(aRay->GetDirection()); //Cos(Theta_i)
	//Ci = (Ci > 0) ? Ci : -Ci;
	float St2 = (refractive_index_i/refractive_index_e)*(refractive_index_i/refractive_index_e)*(1-Ci*Ci);
	bool total_internal = (St2 > 1) ? true : false;
	
	float Ct = (!total_internal) ? sqrt(1-St2) : 0;
	float S_contr = (refraction_flag) ? GetReflectance(Ci,Ct,refractive_index_e,refractive_index_i) : specular_contribution;
	float R_contr = 1 - S_contr;

	/* Specular Contribution
	*/
	if(specular_flag)
	{
		std::shared_ptr<IRay> aTempRay(new Ray());
		ReflectedRays.push_back(aTempRay);

		Eigen::Vector3f ref_dir;

		ref_dir = aRay->GetDirection() - 2*(normalU.dot(aRay->GetDirection()))*normalU;

		ReflectedRays.back()->SetDirection(ref_dir);
		ReflectedRays.back()->SetIntensity(aRay->GetIntensity()*S_contr);
		ReflectedRays.back()->SetReflectionlimite(aRay->GetReflectionCounter()-1);

	#ifdef DELTA
		ReflectedRays.back()->SetPoint(a.GetCollisionPosition() + DELTA*ref_dir);
	#else
		ReflectedRays.back()->SetPoint(a.GetCollisionPosition());
	#endif //DELTA != 0
	}

	/* Diffuse Contribution
	*/
	if(diffuse_flag)
	{
		int flag = 1;
		float gain;
		vector<Ilight*> lights = aScene->Getlight();
		std::vector<Ilight*>::iterator begin = lights.begin();
		for(std::vector<Ilight*>::iterator it = lights.begin(); it != lights.end(); ++it)
		{
			std::shared_ptr<IRay> aTempRayD(new Ray(a.GetCollisionPosition(),(*it)->GetCenter()-a.GetCollisionPosition(),aRay->GetIntensity(),aRay->GetReflectionCounter()-1,it-begin));
			ReflectedRays.push_back(aTempRayD);
			
			//Checking if the light is in the same side as the origin ray, only if non-refractive
			if(!refraction_flag)
			{
				flag = 0;
				if(aRay->GetDirection().dot(normal) < 0)
				{
					if(ReflectedRays.back()->GetDirection().dot(normal) > 0) flag = 1;
				}
				else
					if(ReflectedRays.back()->GetDirection().dot(normal) < 0) flag = 1;
			}

			if(aScene->Collision(ReflectedRays.back()) || flag == 0 ) //If there isn't a direct path to the light, remove last added ray
			{ 
				ReflectedRays.pop_back();
			}
			else
			{
				gain = ReflectedRays.back()->GetDirection().dot(normal);
				gain = ( (gain >= 0) ? gain : 0);
				ReflectedRays.back()->SetIntensity(TWOWAY_MUL*lambertian_contribution*gain*colorIntensity.cwiseProduct( ReflectedRays.back()->GetIntensity() ) );
				
				#ifdef DELTA
					ReflectedRays.back()->SetPoint(ReflectedRays.back()->GetPoint()+ReflectedRays.back()->GetDirection()*DELTA);  
				#endif // DELTA != 0

				#ifdef TWOWAY
					std::shared_ptr<IRay> aTempRay2(new Ray(ReflectedRays.back()->GetPoint(),ReflectedRays.back()->GetDirection(),ReflectedRays.back()->GetIntensity(),ReflectedRays.back()->GetReflectionCounter(),-1));
					ReflectedRays.push_back(aTempRay2);
				#endif //TWOWAY
			}
		}
	}

	/* Refractive Contribution
	*/
	if(refraction_flag && !total_internal)
	{

		float r = refractive_index_e*refractive_index_e - refractive_index_i*refractive_index_i;
		float c = -normalU.dot(aRay->GetDirection());
		Eigen::Vector3f refrac_dir;

		refrac_dir = aRay->GetDirection()  + c*normalU - std::sqrt( r + c*c)*normalU;

		#ifdef DELTA
			std::shared_ptr<IRay> aTempRayr(new Ray(a.GetCollisionPosition() + refrac_dir*DELTA,refrac_dir,aRay->GetIntensity()*R_contr,aRay->GetReflectionCounter()-1));
		#else
			std::shared_ptr<IRay> aTempRayr(new Ray(a.GetCollisionPosition(),refrac_dir,aRay->GetIntensity()*R_contr,aRay->GetReflectionCounter()-1));
		#endif //DELTA

		ReflectedRays.push_back(aTempRayr);
	}

	return ReflectedRays;
}