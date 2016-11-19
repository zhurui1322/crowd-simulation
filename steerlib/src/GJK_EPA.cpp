/*!
*
* \author Rui Zhu, Qian Wu, Yun Wang, Yi Yu
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	std::vector<std::vector<Util::Vector>> setoftrianglesA;
	std::vector<std::vector<Util::Vector>> setoftrianglesB;
	separatepolygon(_shapeA,setoftrianglesA);
	separatepolygon(_shapeB,setoftrianglesB);
	
	for(int i = 0; i<setoftrianglesA.size(); i++)
	{
		for(int j = 0; j<setoftrianglesB.size(); j++)
		{
			//std::cerr <<"A set "<<setoftrianglesA[i][0]<<setoftrianglesA[i][1]<<setoftrianglesA[i][2] <<std::endl;
			//std::cerr <<"B set "<<setoftrianglesB[j][0]<<setoftrianglesB[j][1]<<setoftrianglesB[j][2] <<std::endl;
			if(GJK(setoftrianglesA[i],setoftrianglesB[j], simplex))
			{
				//std::cerr <<"collision"<<setoftrianglesA[i][0] <<std::endl;

				simplex.clear();
				if(GJK(_shapeA, _shapeB, simplex))
				{
					EPA(return_penetration_depth,return_penetration_vector,_shapeA,_shapeB, simplex);
					return true;
				}

			}
			simplex.clear();
		}
	}
	return false;
	
	/*
	if(GJK(_shapeA, _shapeB, simplex))
	{
		EPA(return_penetration_depth,return_penetration_vector,_shapeA, _shapeB, simplex);
		return true;
	}	
	else
		return false;
	*/
}
void SteerLib::GJK_EPA::separatepolygon(std::vector<Util::Vector> shape, std::vector<std::vector<Util::Vector>>& setoftriangles)
{
	std::vector<Util::Vector> newtriangle, newpolygonA, newpolygonB;	
	int leftmostIndex;
	int leftmostIndexadd1;
	int leftmostIndexminus1;
	int count;
	Util::Vector leftmostDirection;
	int loopsize = shape.size()-2;
	leftmostDirection.x = -1;
	leftmostDirection.y = 0;
	leftmostDirection.z = -0.001;
	bool isPointIn = false;
	for(int i = 0; i < loopsize; i++)
	{
		
		leftmostIndex = findfarthestPointindexDirection(shape, leftmostDirection);
		//std::cerr <<"the left most index "<<leftmostIndex<<std::endl;
		leftmostIndexadd1 = leftmostIndex+1 == shape.size() ? 0 : leftmostIndex+1;
		leftmostIndexminus1 = leftmostIndex-1 == -1 ? shape.size()-1 :leftmostIndex-1 ;
		
		//std::cerr <<"the left most index+"<<leftmostIndexadd1<<std::endl;
		//std::cerr <<"the left most index-"<<leftmostIndexminus1<<std::endl;
		newtriangle.push_back(shape[leftmostIndexadd1]);
		newtriangle.push_back(shape[leftmostIndex]);
		newtriangle.push_back(shape[leftmostIndexminus1]);
		
		if(shape.size()<=3)
		{
			setoftriangles.push_back(newtriangle);
			//std::cerr <<"the new triangle is <3 "<<newtriangle[0]<<newtriangle[1]<<newtriangle[2]<<std::endl;
			newtriangle.clear();
					
		}
		else
		{
			for(int j = 0; j<shape.size(); j++)
			{
				
				if(shape[j] != newtriangle[0]&&shape[j] != newtriangle[1]&&shape[j] != newtriangle[2])
				{			
					//std::cerr <<"here "<<shape[j]<<std::endl;
					if(PointInTri(shape[j], newtriangle[0], newtriangle[1], newtriangle[2]))
					{
						//std::cerr <<"point is in triangle "<<shape[j]<<std::endl;
						count = leftmostIndex;
						isPointIn = true;
						while(count!=j)
						{
							newpolygonA.push_back(shape[count]);
							count = count+1 == shape.size() ? 0 : count+1;
						}
						newpolygonA.push_back(shape[j]);

						count = leftmostIndex;
						while(count!=j)
						{
							newpolygonB.push_back(shape[count]);
							count = count-1 == -1 ? shape.size()-1 : count-1;
						}
						newpolygonB.push_back(shape[j]);
						separatepolygon(newpolygonA, setoftriangles);
						separatepolygon(newpolygonB, setoftriangles);
						newpolygonB.clear();
						newpolygonA.clear();
						//std::cerr <<"here "<<shape[j]<<std::endl;
						break;
					}			
				}
			}
			if(!isPointIn)
			{
				setoftriangles.push_back(newtriangle);
				//std::cerr <<"the new triangle is noin"<<newtriangle[0]<<newtriangle[1]<<newtriangle[2]<<std::endl;
				newtriangle.clear();
				shape.erase(shape.begin()+leftmostIndex);
			}else{break;}
		}	
		isPointIn = false;
	}

}

bool SteerLib::GJK_EPA::PointInTri(Util::Vector point, Util::Vector p1, Util::Vector p2, Util::Vector p3)
{
	bool b1,b2,b3;
	b1 = signTriangle(point,p1,p2) < 0.0f;
	b2 = signTriangle(point,p2,p3) < 0.0f;
	b3 = signTriangle(point,p3,p1) < 0.0f;
	return ((b1 == b2) && (b2 == b3));


}
float SteerLib::GJK_EPA::signTriangle(Util::Vector p1, Util::Vector p2, Util::Vector p3)
{
	return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
}


int SteerLib::GJK_EPA::findfarthestPointindexDirection(const std::vector<Util::Vector>& shape, const Util::Vector direction)
{
	int farthestIndex = 0;
	float farthestDistance = Dotproduct(shape[0],direction);
	float temp;
	for(int i = 0; i < shape.size(); i++)
	{
		temp = Dotproduct(shape[i],direction);
		if(temp >= farthestDistance)
		{
			farthestDistance = temp;
			farthestIndex = i;
		}
	}
	return farthestIndex;
}



void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	int index;
	float distanceE;
	float distance;
	Util::Vector normal;
	Util::Vector newPoint;
	
	while(true)
	{
		findClosestEdge(index, distanceE, normal, simplex);
		//std::cerr <<"normal is  "<<normal<<std::endl;
		//std::cerr <<"index is  "<<index<<std::endl;
		//std::cerr <<"distance is  "<<distance<<std::endl;
		newPoint = supportEPA(_shapeA, _shapeB, normal);
		//std::cerr <<"newpoint is  "<<newPoint<<std::endl;
		distance = Dotproduct(newPoint,normal);
		if(distance - distanceE < 0.0001)
		{
			return_penetration_depth = distance;
			return_penetration_vector = negate(normal);
			return;
		}	
		else
		{
			simplex.insert(simplex.begin()+index, newPoint);
		}
	}	
}



void  SteerLib::GJK_EPA::findClosestEdge(int& index, float& distance, Util::Vector& normal, const std::vector<Util::Vector>& simplex)
{	
	Util::Vector pointA;
	Util::Vector pointB;
	Util::Vector pointE;
	Util::Vector lineA0;
	Util::Vector N;
	float predistance;

	distance = std::numeric_limits<float>::max();

	for(int i = 0; i<simplex.size(); i++)
	{
		int j = i+1 == simplex.size() ? 0 : i+1;
		pointA = simplex[i];
		pointB = simplex[j];
		
		pointE.x = pointB.x - pointA.x;
		pointE.z = pointB.z - pointA.z;
		pointE.y = 0;
		
		lineA0 = pointA;
		N = tripleproduct(pointE, lineA0, pointE);
		//std::cerr <<"N is  "<<N<<std::endl;
		N = normalize(N);
		//std::cerr <<"N after normalize is  "<<N<<std::endl;
		predistance = Dotproduct(N, pointA);
		
		if(predistance < distance)
		{
			distance = predistance;
			normal = N;
			index = i+1;	
		}
	}
}



Util::Vector SteerLib::GJK_EPA::normalize(Util::Vector vector)
{
	Util::Vector normal;
	float length;
	length = sqrt(vector.x * vector.x + vector.z * vector.z); 
	normal.x = vector.x / length;
	normal.z = vector.z / length;
	normal.y = 0;
	return normal;
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	Util::Vector direction;
	Util::Vector Newpoint;
	Util::Vector AB;
	Util::Vector A0;
	Util::Vector tripleProduct;
	
	float proj;

	//find the center of each polygon to decide the initial direction 
	Util::Vector centerA;
	Util::Vector centerB;
	centerA = CenterOfPolygon(_shapeA);
	centerB = CenterOfPolygon(_shapeB);

	/*std::cerr <<"the A point is "<<_shapeA[0]<<std::endl;
	std::cerr <<"the A point is "<<_shapeA[1]<<std::endl;
	std::cerr <<"the A point is "<<_shapeA[2]<<std::endl;
	std::cerr <<"the A point is "<<_shapeA[3]<<std::endl;
	std::cerr <<"the A point is "<<_shapeA[4]<<std::endl;
	std::cerr <<"center A is "<<centerA<<std::endl;
	std::cerr <<std::endl;
	std::cerr <<"the B point is "<<_shapeB[0]<<std::endl;
	std::cerr <<"the B point is "<<_shapeB[1]<<std::endl;
	std::cerr <<"the B point is "<<_shapeB[2]<<std::endl;
	std::cerr <<"the B point is "<<_shapeB[3]<<std::endl;
	std::cerr <<"the B point is "<<_shapeB[4]<<std::endl;
	std::cerr <<"center B is "<<centerB<<std::endl;*/

	direction.x = centerA.x - centerB.x;
	direction.y = 0;
	direction.z = centerA.z - centerB.z;
	Newpoint = support(_shapeA, _shapeB, direction);
	simplex.push_back(Newpoint);
	direction = negate(direction);
	while(true)
	{
		Newpoint = support(_shapeA, _shapeB, direction);
		simplex.push_back(Newpoint);
		proj = Dotproduct(simplex.back(),direction);	
		if(proj <= 0)
		{
			return false;
		}
		else
		{
			if(containsOrigin(simplex, direction))
				return true;
		}
	}
}



bool SteerLib::GJK_EPA::containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction)
{
	Util::Vector pointA, pointB, pointC;
	Util::Vector lineA0, lineAB, lineAC;
	Util::Vector ABperp, ACperp;

	pointA = simplex.back();
	lineA0 = negate(pointA);
	if(simplex.size() == 3)
	{
		pointB = simplex[1];
		pointC = simplex[0];
		
		lineAB.x = pointB.x - pointA.x;
		lineAB.z = pointB.z - pointA.z;
		lineAB.y = 0;

		lineAC.x = pointC.x - pointA.x;
		lineAC.z = pointC.z - pointA.z;
		lineAC.y = 0;

		ABperp = tripleproduct(lineAC, lineAB, lineAB);
		ACperp = tripleproduct(lineAB, lineAC, lineAC);
		if(Dotproduct(ABperp, lineA0) > 0)
		{
			simplex.erase(simplex.begin());
			direction = ABperp;	
		}
		else
		{
			if(Dotproduct(ACperp, lineA0) > 0)
			{	
				simplex.erase(simplex.begin()+1);
				direction = ACperp;
			}
			else
			{
				return true;
			}	
		}		
	}
	else
	{
		pointB = simplex[0];
		lineAB.x = pointB.x - pointA.x;
		lineAB.z = pointB.z - pointA.z;
		lineAB.y = 0;
		ABperp = tripleproduct(lineAB, lineA0, lineAB);
		direction = ABperp;
	}
	return false;
}



Util::Vector SteerLib::GJK_EPA::negate(const Util::Vector vector)
{
	Util::Vector negatevector;
	negatevector.x = -vector.x;
	negatevector.z = -vector.z;
	negatevector.y = 0;
	return negatevector;
}



Util::Vector SteerLib::GJK_EPA::tripleproduct(const Util::Vector lineA, const Util::Vector lineB,const Util::Vector lineC)
{
	Util::Vector tripleProduct;
	tripleProduct.x = lineB.x * Dotproduct(lineC,lineA) - lineA.x * Dotproduct(lineC,lineB);
	tripleProduct.z = lineB.z * Dotproduct(lineC,lineA) - lineA.z * Dotproduct(lineC,lineB);
	tripleProduct.y = 0;

	return tripleProduct;
}



Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, Util::Vector direction)
{	
	Util::Vector point1;
	Util::Vector point2;
	Util::Vector point3;	

	point1 = findfarthestPointinDirection(shapeA, direction);
	point2 = findfarthestPointinDirection(shapeB, negate(direction));

	point3.x = point1.x - point2.x;
	point3.z = point1.z - point2.z+0.001;
	point3.y = 0;
	return point3;
}



Util::Vector SteerLib::GJK_EPA::supportEPA(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, Util::Vector direction)
{	
	Util::Vector point1;
	Util::Vector point2;
	Util::Vector point3;	

	point1 = findfarthestPointinDirection(shapeA, direction);
	point2 = findfarthestPointinDirection(shapeB, negate(direction));

	point3.x = point1.x - point2.x;
	point3.z = point1.z - point2.z;
	point3.y = 0;
	return point3;
}



Util::Vector SteerLib::GJK_EPA::findfarthestPointinDirection(const std::vector<Util::Vector>& shape, Util::Vector direction)
{
	int farthestIndex = 0;
	float farthestDistance = Dotproduct(shape[0],direction);
	float temp;
	for(int i = 0; i < shape.size(); i++)
	{
		temp = Dotproduct(shape[i],direction);
		if(temp >= farthestDistance)
		{
			farthestDistance = temp;
			farthestIndex = i;
		}
	}
	return shape[farthestIndex];
}



float SteerLib::GJK_EPA::Dotproduct(const Util::Vector vector1, Util::Vector vector2)
{
	return vector1.x * vector2.x + vector1.z * vector2.z;
}



Util::Vector SteerLib::GJK_EPA::CenterOfPolygon(const std::vector<Util::Vector>& shape)
{
	Util::Vector center;
	float Aval = 0;
	float step2sum = 0;
	int count = shape.size()-1;	

	for(int i = 0; i < shape.size()-1;i++)
	{
		Aval += (shape[i].x * shape[i+1].z) - (shape[i+1].x * shape[i].z);
	}
	Aval +=  (shape[count].x * shape[0].z) - (shape[0].x * shape[count].z);
	Aval = Aval/2;

	for(int i= 0; i < shape.size()-1;i++)
	{
		step2sum += (shape[i].x + shape[i+1].x)*((shape[i].x * shape[i+1].z) - (shape[i+1].x * shape[i].z));
	}
	step2sum += (shape[count].x + shape[0].x)*((shape[count].x * shape[0].z) - (shape[0].x * shape[count].z));
	
	step2sum = step2sum/(Aval*6);
 	center.x = step2sum;
	step2sum = 0;

	for(int i= 0; i < shape.size()-1;i++)
	{
		step2sum += (shape[i].z + shape[i+1].z)*((shape[i].x * shape[i+1].z) - (shape[i+1].x * shape[i].z));
	}
	step2sum += (shape[count].z + shape[0].z)*((shape[count].x * shape[0].z) - (shape[0].x * shape[count].z));
	step2sum = step2sum/(Aval*6);
 	center.z = step2sum;
	center.y = 0;
	return center;
}
