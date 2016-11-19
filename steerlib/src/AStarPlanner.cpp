//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id, double OBSTACLE_CLEARANCE ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path, double OBSTACLE_CLEARANCE )
	{
		gSpatialDatabase = _gSpatialDatabase;
		
		std::vector<AStarPlannerNode> openList;
		std::vector<AStarPlannerNode> closedList;

		
		Util::Point adjPoint, currentPoint;
		Util::Point startP, goalP;
		unsigned int startCellIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int goalCellIndex = gSpatialDatabase->getCellIndexFromLocation(goal);
		gSpatialDatabase->getLocationFromIndex( startCellIndex, startP);
		gSpatialDatabase->getLocationFromIndex( goalCellIndex, goalP);
		


		
		int adjPoint_ID;
		double Hvalue, Fvalue;
		double GValue = 0;
		int lowestIndex;
		int weight = 2;
		int or48;
		int nodeinopenIndex;

		
		if (append_to_path)
			or48 = 8;
		else
			or48 = 4;
		Hvalue = weight * calculateHvalueManhattan(startP, goalP);
		Fvalue = 0 + Hvalue;
		AStarPlannerNode node(startP,GValue,Fvalue, -1);
		openList.push_back(node);
		//printlistcontent(openList);

		while(!openList.empty())
		
		{
			lowestIndex = findlowestF(openList);
			
			if(openList[lowestIndex].point.x == goalP.x &&openList[lowestIndex].point.z == goalP.z )
			{
				construct_path(agent_path, start, closedList, openList[lowestIndex]);
				
				//std::cerr <<"find goal"<<"expanded nodes number is "<<closedList.size()<<std::endl;
				return true;
			}
			closedList.push_back(openList[lowestIndex]);
			
			for(int i = 0; i < or48; i++)
			{
				if(i == 0)
				{
					adjPoint.x = openList[lowestIndex].point.x + 1;
					adjPoint.z = openList[lowestIndex].point.z;
					GValue = 1; 
				}
				if(i == 1)
				{
					adjPoint.x = openList[lowestIndex].point.x - 1;
					adjPoint.z = openList[lowestIndex].point.z;
					GValue = 1;  
				}
				if(i == 2)
				{
					adjPoint.x = openList[lowestIndex].point.x;
					adjPoint.z = openList[lowestIndex].point.z + 1; 
					GValue = 1; 
				}
				if(i == 3)
				{
					adjPoint.x = openList[lowestIndex].point.x;
					adjPoint.z = openList[lowestIndex].point.z - 1; 
					GValue = 1; 
				}
				if(i == 4)
				{
					adjPoint.x = openList[lowestIndex].point.x + 1;
					adjPoint.z = openList[lowestIndex].point.z + 1; 
					//GValue = 1; 					
					GValue = 1.4142; 
				}
				if(i == 5)
				{
					adjPoint.x = openList[lowestIndex].point.x - 1;
					adjPoint.z = openList[lowestIndex].point.z - 1; 
					//GValue = 1; 					
					GValue = 1.4142; 
				}
				if(i == 6)
				{
					adjPoint.x = openList[lowestIndex].point.x + 1;
					adjPoint.z = openList[lowestIndex].point.z - 1; 
					//GValue = 1; 
					GValue = 1.4142; 
				}
				if(i == 7)
				{
					adjPoint.x = openList[lowestIndex].point.x - 1;
					adjPoint.z = openList[lowestIndex].point.z + 1; 
					//GValue = 1; 
					GValue = 1.4142; 
				}


				
				adjPoint_ID = gSpatialDatabase->getCellIndexFromLocation(adjPoint.x, adjPoint.z);
				if(canBeTraversed(adjPoint_ID, OBSTACLE_CLEARANCE))
				{
					if(!isnodeinlist(closedList, adjPoint))
					{
						if(!isnodeinlist(openList, adjPoint))
						{
							Hvalue = weight*calculateHvalueManhattan(adjPoint, goal);
							GValue = openList[lowestIndex].g + 1.4142; ;
							Fvalue = Hvalue + GValue;
							AStarPlannerNode node1(adjPoint,GValue,Fvalue, closedList.size()-1);
							//std::cerr <<"node is" <<node1.point.x<<","<<node1.point.z<<" parent is "<<node1.->point.x<<","<<node1.parent->point.z<<std::endl;
							openList.push_back(node1);
							
						}
						else
						{
							 nodeinopenIndex = findnodeinopenlist(openList, adjPoint);
							 if(openList[nodeinopenIndex].g > openList[lowestIndex].g)
							 {
							 	//std::cerr <<"here" <<std::endl;
							 	openList[nodeinopenIndex].g = openList[lowestIndex].g;
							 	openList[nodeinopenIndex].index = openList[lowestIndex].index;
							 }
						}
					}
				}
					
			}
			openList.erase(openList.begin()+lowestIndex);
			closedList.push_back(openList[lowestIndex]);
		}


		return false;
	}

	void AStarPlanner::construct_path(std::vector<Util::Point>& agent_path, Util::Point start, std::vector<AStarPlannerNode> List, AStarPlannerNode node)
	{
		int parentindex;
		int length = 1;
		agent_path.push_back(node.point);
		//std::cerr <<"node is "<<node.point.x<<","<<node.point.z<<" g is"<<node.g<<std::endl;
		parentindex = node.index;
		//std::cerr <<"node is "<<List[parentindex].point.x<<","<<List[parentindex].point.z<<" g is"<<List[parentindex].g<<std::endl;
		agent_path.insert(agent_path.begin(), List[parentindex].point);
		length++;

		while(List[parentindex].index != -1){
			parentindex = List[parentindex].index;
			//std::cerr <<"node is "<<List[parentindex].point.x<<","<<List[parentindex].point.z<<" g is"<<List[parentindex].g<<std::endl;
			agent_path.insert(agent_path.begin(), List[parentindex].point);
			length++;
		}
		//std::cerr <<"path length is " <<length<<std::endl;


		return;
	}


	void AStarPlanner::printlistcontent(std::vector<AStarPlannerNode> List)
	{
		for(int i = 0; i < List.size(); i++)
		{
			std::cerr <<List[i].point.x<<","<<List[i].point.z<<"f value is "<<List[i].f<<std::endl;
		}
	}

	int AStarPlanner::findnodeinopenlist(std::vector<AStarPlannerNode> List, Util::Point point)
	{
		for(int i = 0; i < List.size(); i++)
		{
			if((point.x == List[i].point.x) && (point.z == List[i].point.z))
				return i;
		}
	}


	bool AStarPlanner::isnodeinlist(std::vector<AStarPlannerNode> List, Util::Point point)
	{
		for(int i = 0; i < List.size(); i++)
		{
			if((point.x == List[i].point.x) && (point.z == List[i].point.z))
				return true;
		}
		return false;
	}



	int AStarPlanner::findlowestF(std::vector<AStarPlannerNode> List)
	{
		int lowestIndex = 0;


		if(List.size() == 1)
		{
			return lowestIndex;
		}

		for(int i = 0; i < List.size(); i++)
		{
			if(List[lowestIndex].f > List[i].f)
				lowestIndex = i;
			if(List[lowestIndex].f == List[i].f)
                        {
				if(List[lowestIndex].g < List[i].g)
				{
					lowestIndex = i;
				}
			}
		}
		return lowestIndex;

	}

	double AStarPlanner::calculateHvalueManhattan(Util::Point point1, Util::Point point2)
	{
		return abs(point2.x - point1.x) +  abs(point2.z - point1.z);
	}
	

	double AStarPlanner::calculateHvalueEuclidean(Util::Point point1, Util::Point point2)
	{
		return sqrt((point2.x -point1.x)*(point2.x -point1.x) + (point2.z -point1.z)*(point2.z -point1.z));
	}
}
