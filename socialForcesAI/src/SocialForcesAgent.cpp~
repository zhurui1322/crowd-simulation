//
// Copyright (c) 2014-2015 VaHiD aZiZi
//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
#include "obstacles/PolygonObstacle.h"
#include "obstacles/GJK_EPA.h"
#include "testcaseio/AgentInitialConditions.h"
// #include <math.h>


// #include "util/Geometry.h"


/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.


#undef min
#undef max
//#define _DEBUG_

#define AGENT_MASS 1.0f


using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;


// #define _DEBUG_ENTROPY 1


SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}


SocialForcesAgent::~SocialForcesAgent()
{
}


void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}


void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled==true);

	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}


void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;
	_waypoints.clear();
	_midTermPath.clear();

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	// std::cout << "radius of agent " << _radius << std::endl;
	double OBSTACLE_CLEARANCE;
	bool or48 = true;
	if(_radius == 1){
		or48 = false;
		OBSTACLE_CLEARANCE = 2;
	}
	else{
		OBSTACLE_CLEARANCE = 0; 
		or48 = true;
	}
	
	
	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		gSpatialDatabase->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				_goalQueue.back().targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
			}
		}
		else {
		
	throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}

	}

/*********************************************************************************************************************************************/
/*********************************************************************************************************************************************/
/*********************************************************************************************************************************************/
	
	std::vector<Util::Point> a_path;
	Util::Point final_goal = _goalQueue.front().targetLocation;
	
	//std::cout<<"goalqueue size "<<_goalQueue.size()<<"final goal is "<< _goalQueue.front().targetLocation<<std::endl;
	_goalQueue.pop();	
	//std::cout<<"goalqueue size "<<_goalQueue.size()<<std::endl;
	astar.computePath(a_path, _position, final_goal, gSpatialDatabase, or48, OBSTACLE_CLEARANCE);
	
	for  (int i = 1; i < a_path.size();  i++)
	{
		_midTermPath.push_back(a_path[i]);
		_waypoints.push_back(a_path[i]);
		SteerLib::AgentGoalInfo goal_temp;
		goal_temp.targetLocation = a_path[i];
		
		_goalQueue.push(goal_temp);
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			//_waypoints.push_back(a_path.at(i));
		}
	}
	_midTermPath.push_back(final_goal);
	_waypoints.push_back(final_goal);
	SteerLib::AgentGoalInfo goal_temp;
	goal_temp.targetLocation = final_goal;
		
	_goalQueue.push(goal_temp);
	
	/*std::cout<<"goalqueue size "<<_goalQueue.size()<<std::endl;
	std::cout<<"waypoint  size "<<_waypoints.size()<<std::endl;
	std::cout<<"midtermPath  size "<<_midTermPath.size()<<std::endl;


	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	if ( !_midTermPath.empty() )
	{
		this->updateLocalTarget();
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						PREFERED_SPEED
					)
				- velocity()
				)
				/
				_SocialForcesParams.sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif

	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void SocialForcesAgent::calcNextStep(float dt)
{
    //nothing to do here
}


std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
    //std::cerr<<"<<<calcProximityForce>>> Please Implement my body\n";
	Util::Vector proximity_force = Util::Vector(0,0,0);
	
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; 
	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	gEngine->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent; 
	SteerLib::ObstacleInterface * tmp_ob; 
	Util::Vector away = Util::Vector(0,0,0);
	Util::Vector away_obs = Util::Vector(0,0,0);
    for(std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
    {
        if( (*neighbour) -> isAgent() ) // exclude obstacle
        {
            tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour); 

            Util::Vector away_tmp = normalize(position() - tmp_agent->position()); 
            away = away + away_tmp * _SocialForcesParams.sf_agent_a * 
                exp( 
                        ((this->radius() + tmp_agent->radius()) - (this->position() - tmp_agent->position()).length())/_SocialForcesParams.sf_agent_b
                   )
                * dt;
        }
        else
        {

            tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
            Util::Vector wall_normal = calcWallNormal( tmp_ob ); 
            std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
            std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position()); 

            Util::Vector away_obs_tmp = normalize(position() - min_stuff.second); 
            away_obs = away_obs + away_obs_tmp * _SocialForcesParams.sf_wall_a * 
                exp(
                        ( this->radius() - min_stuff.first)/_SocialForcesParams.sf_wall_b
                   )
                * dt; 
        }
    }
    //proximity_force = ((away + away_obs)/ AGENT_MASS)* dt; 
    proximity_force = away + away_obs; 
    return proximity_force; 
}


Vector SocialForcesAgent::calcGoalForce(Vector _goalDirection, float _dt)
{
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; 
	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	gEngine->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::AgentInterface *tmp_agent;
    Util::Vector neighbor_avg(0,0,0);
    std::vector<SteerLib::AgentInterface *> neighbor_agents;

    for(std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++){
        if((*neighbour)->isAgent()){
            tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour); 
        }
        else{
            continue;
        }
        if(id() != tmp_agent->id()){
            neighbor_agents.push_back(tmp_agent);
        }   
    }

    for(std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++){
        neighbor_avg += normalize((*tmp_ag)->currentGoal().targetLocation - (*tmp_ag)->position());
    }

    float pi = 0.3f;
    if(!neighbor_agents.empty()){
        neighbor_avg = neighbor_avg / neighbor_agents.size();
    }
    Util::Vector _direction = normalize((1-pi)*_goalDirection + pi*neighbor_avg);

	Util::Vector goal_force = _dt * AGENT_MASS * (_direction * PREFERED_SPEED - velocity()) / _dt;

    return 4*goal_force; 
}


Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
			(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + ( _SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) + calcPolygonRepulsionForce(dt);
}


Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{
    Util::Vector agent_repulsion_force = Util::Vector(0,0,0);
	
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; 
	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	gEngine->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::AgentInterface *tmp_agent; 
    for(std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
    {
        if( (*neighbour) -> isAgent() ) // exclude obstacle
        {
            tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour); 
        }
        else
        {
            continue; 
        }

        if( (id() != tmp_agent->id()) && (tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001f))
        {
        
            Util::Vector temp_agent_repulsion = tmp_agent->computePenetration(this->position(), this->radius()) * _SocialForcesParams.sf_agent_body_force * normalize(position() - tmp_agent->position()) * dt; 

            Util::Vector nij = normalize(this->_position - tmp_agent->position());
            Util::Vector tij = Util::Vector(-nij.z, 0.0f, nij.x);
            float vij = (tmp_agent->velocity() - this->_velocity) * tij;

            Util::Vector temp_sliding = tmp_agent->computePenetration(this->position(),this->radius()) * _SocialForcesParams.sf_sliding_friction_force * vij * tij * dt;

//            agent_repulsion_force += (temp_agent_repulsion + temp_sliding);
            agent_repulsion_force += temp_agent_repulsion + temp_sliding;
        }

    }
    //agent_repulsion_force = (agent_repulsion_force / AGENT_MASS)* dt; 

    return agent_repulsion_force;
}


Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{
    //std::cerr<<"<<<calcWallRepulsionForce>>> Please Implement my body\n";
	Util::Vector wall_repulsion_force = Util::Vector(0,0,0);
	std::vector<Util::Vector> wall_repulsion_force_list; 
	//SteerLib::EngineInterface * engineInfo; 

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; 
	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	gEngine->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::ObstacleInterface *tmp_ob; 
    for(std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
    {
        if(!(*neighbour) -> isAgent()) // exclude agents
        {
            tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour); 
        }
        else
        {
            continue; 
        }

        if(tmp_ob->computePenetration(this->position(), this->radius())> 0.000001f)
        {
            Util::Vector wall_repulsion_force_tmp(0,0,0); 
            Util::Vector wall_normal = calcWallNormal( tmp_ob ); 
            std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
            std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position()); 

            wall_repulsion_force_tmp = wall_normal * (radius() - min_stuff.first) * _SocialForcesParams.sf_body_force * dt;
//            wall_repulsion_force_list.push_back(wall_repulsion_force_tmp); 
            
            Util::Vector tiw = Util::Vector(-wall_normal.z, 0.0f, wall_normal.x);
            Util::Vector wall_sliding = (radius() - min_stuff.first) * _SocialForcesParams.sf_sliding_friction_force * (-velocity()) * tiw * tiw * dt;

//            wall_repulsion_force += (wall_repulsion_force_tmp + wall_sliding);
            wall_repulsion_force += wall_repulsion_force_tmp;

        }

    }
/*
    for(std::vector<Util::Vector>::iterator it = wall_repulsion_force_list.begin(); it != wall_repulsion_force_list.end(); it++)
    {
        wall_repulsion_force += *it; 
    }
*/
//    wall_repulsion_force = (wall_repulsion_force / AGENT_MASS)* dt; 
    return wall_repulsion_force;
}


std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */


Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}


bool SocialForcesAgent::reachedCurrentWaypoint()
{

	if ( !_waypoints.empty())
	{
		return (position() - _waypoints.front()).lengthSquared() <= (radius()*radius());
	}
	else
	{
		false;
	}

	// return (position() - _currentLocalTarget).lengthSquared() < (radius()*radius());
}


bool SocialForcesAgent::hasLineOfSightTo(Util::Point target)
{
	float dummyt;
	Util::Vector _rightSide = rightSideInXZPlane(this->forward());
	SpatialDatabaseItemPtr dummyObject;
	Ray lineOfSightTestRight, lineOfSightTestLeft;
	// lineOfSightTestRight.initWithUnitInterval(_position + _radius*_rightSide, target - (_position + _radius*_rightSide));
	// lineOfSightTestLeft.initWithUnitInterval(_position + _radius*(_rightSide), target - (_position - _radius*_rightSide));
	lineOfSightTestRight.initWithUnitInterval(_position + _radius*_rightSide, target - _position);
	lineOfSightTestLeft.initWithUnitInterval(_position + _radius*(_rightSide), target - _position);

	return (!gSpatialDatabase->trace(lineOfSightTestRight,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true))
		&& (!gSpatialDatabase->trace(lineOfSightTestLeft,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true));

}


void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// std::cout << "_SocialForcesParams.rvo_max_speed " << _SocialForcesParams._SocialForcesParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &SocialForcesGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	
	Util::Vector goalDirection;

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();





	if ( ! _midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)) )
	{
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}

		this->updateLocalTarget();

		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}


/**************************************************************************************************************************************/
/**************************************************************************************************************************************/
/***************************************************************************************************************************************/
		double OBSTACLE_CLEARANCE;
	bool or48 = true;
	if(_radius == 1){
		OBSTACLE_CLEARANCE = 2;
		//or48 = false;
	}
	else{
		OBSTACLE_CLEARANCE = 0; 
		//or48 = true;
	}



	goalInfo = _goalQueue.front();
	float distance;
	distance = abs(goalInfo.targetLocation.x -position().x) +  abs(goalInfo.targetLocation.z - position().z);
	//std::cout<<"position is "<<position()<<" local goal is "<<goalInfo.targetLocation<<"goal queue size"<<_goalQueue.size()<<std::endl;
	
	if((!this->hasLineOfSightTo(goalInfo.targetLocation) && _velocity.x + _velocity.z < 0.001)|| (distance > 5 && _velocity.x + _velocity.z < 0.1))
	//if(!this->hasLineOfSightTo(goalInfo.targetLocation))
	{
	/*
		if(distance > 5 && _velocity.x + _velocity.z < 0.1)
			std::cout<<"too far away recaul"<<std::endl;
		if((!this->hasLineOfSightTo(goalInfo.targetLocation) && _velocity.x + _velocity.z < 0.001))
			std::cout<<"has line of sight is false"<<std::endl;
		*/
		//std::cout<<"has line of sight to is ture"<< std::endl;
		//std::cout<<"start is " <<position()<<"the target location is "<<_currentLocalTarget<< std::endl;
		//std::cout<<"final gola is "<<_waypoints[_waypoints.size()-1]<<std::endl;
		Util::Point final_goal = _waypoints[_waypoints.size()-1];
		
	
	 	//std::cout << "resetting agent " << this << " and current velocity is: " << _velocity <<std::endl;
		_waypoints.clear();
		_midTermPath.clear();

		while(_goalQueue.size()!=0)
		{
			_goalQueue.pop();
		}

		std::vector<Util::Point> a_path;
	
		//std::cout<<"goalqueue size "<<_goalQueue.size()<<std::endl;
	
		astar.computePath(a_path, _position, final_goal, gSpatialDatabase, false, OBSTACLE_CLEARANCE);
	
		for  (int i = 1; i < a_path.size();  i++)
		{
			//std::cout<<agentPath[i]<<std::endl;
			_midTermPath.push_back(a_path[i]);
			_waypoints.push_back(a_path[i]);
			SteerLib::AgentGoalInfo goal_temp;
			goal_temp.targetLocation = a_path[i];
		
			_goalQueue.push(goal_temp);
			if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
			{
				//_waypoints.push_back(a_path.at(i));
			}
		}
		_midTermPath.push_back(final_goal);
		_waypoints.push_back(final_goal);
		SteerLib::AgentGoalInfo goal_temp;
		goal_temp.targetLocation = final_goal;
		
		_goalQueue.push(goal_temp);
	
		//std::cout<<"goalqueue size "<<_goalQueue.size()<<std::endl;
		//std::cout<<"waypoint  size "<<_waypoints.size()<<std::endl;
		//std::cout<<"midtermPath  size "<<_midTermPath.size()<<std::endl;

		//std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	
	
	Util::Vector goalDirection;

	if ( !_midTermPath.empty() )
	{
		this->updateLocalTarget();
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						PREFERED_SPEED
					)
				- velocity()
				)
				/
				_SocialForcesParams.sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif

	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
	}
/**************************************************************************************************************************************/
/**************************************************************************************************************************************/
/**************************************************************************************************************************************/


    /*
     *  Goal Force
     */
    Util::Vector prefForce = calcGoalForce( goalDirection, dt );

    /*
     *  Repulsion Force
     */
	Util::Vector repulsionForce = calcRepulsionForce(dt);

	if ( repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		// repulsionForce = velocity() * 0;
	}

    /*
     *  Proximity Force
     */
	Util::Vector proximityForce = calcProximityForce(dt);

// #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	// _velocity = _newVelocity;
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}

    Util::Vector acceleration = (prefForce + repulsionForce + proximityForce) / AGENT_MASS;
    _velocity = velocity() + acceleration * dt;
//	_velocity = (prefForce) + repulsionForce + proximityForce;
	// _velocity = velocity() + repulsionForce + proximityForce;

	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y=0.0f;
#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
		_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	 * Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	 */
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
					Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
							goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().lengthSquared() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

}


/**
 * Removes a number of points from the begining of the path
 *
 * After size of path should be old size - FURTHEST_LOCAL_TARGET_DISTANCE
 */
void SocialForcesAgent::updateMidTermPath()
{
	if ( this->_midTermPath.size() < FURTHEST_LOCAL_TARGET_DISTANCE)
	{
		return;
	}
	if ( !_waypoints.empty())
	{
		_waypoints.erase(_waypoints.begin());
	}
	// save good path
	std::vector<Util::Point> tmpPath;
	// std::cout << "midterm path size " << _midTermPath.size() << std::endl;
	// std::cout << "distance between position and current waypoint " << (position() - _waypoints.front()).length() << std::endl;
	for (unsigned int i=(FURTHEST_LOCAL_TARGET_DISTANCE); i < _midTermPath.size();i++ )
	{
		tmpPath.push_back(_midTermPath.at(i));
	}
	_midTermPath.clear();

	for (unsigned int i=0; i < tmpPath.size(); i++)
	{
		_midTermPath.push_back(tmpPath.at(i));
	}

}


/**
 * Update the local target to the furthest point on the midterm path the agent can see.
 */
void SocialForcesAgent::updateLocalTarget()
{
	Util::Point tmpTarget = this->_goalQueue.front().targetLocation;
	unsigned int i=0;
	for (i=0; (i < FURTHEST_LOCAL_TARGET_DISTANCE) &&
			i < this->_midTermPath.size(); i++ )
	{
		tmpTarget = this->_midTermPath.at(i);
		if ( this->hasLineOfSightTo(tmpTarget) )
		{
			this->_currentLocalTarget = tmpTarget;
		}
	}
}


/**
 * finds a path to the current goal
 * puts that path in midTermPath
 */
bool SocialForcesAgent::runLongTermPlanning()
{
	_midTermPath.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();

	if ( !gSpatialDatabase->findPath(pos, _goalQueue.front().targetLocation,
			agentPath, (unsigned int) 50000))
	{
		return false;
	}

	for  (int i=1; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	return true;
}


bool SocialForcesAgent::runLongTermPlanning2()
{

#ifndef USE_PLANNING
	return;
#endif
	_waypoints.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();
	if (gEngine->isAgentSelected(this))
	{
		// std::cout << "agent" << this->id() << " is running planning again" << std::endl;
	}

	if ( !gSpatialDatabase->findSmoothPath(pos, _goalQueue.front().targetLocation,
			agentPath, (unsigned int) 50000))
	{
		return false;
	}

	// Push path into _waypoints

	// Skip first node that is at location of agent
	for  (int i=1; i <  agentPath.size(); i++)
	{
		_waypoints.push_back(agentPath.at(i));

	}

	return true;

}

Util::Vector SocialForcesAgent::calcPolygonRepulsionForce(float dt)
{
	//std::cout<<"let's restart again"<<std::endl;
	Util::Vector obs_repulsion_force = Util::Vector(0,0,0);
	Util::Vector obs_sliding_force = Util::Vector(0,0,0);
	Util::Vector obs_proximity_force = Util::Vector(0,0,0);
	// what is we need?
	// get set of obstacles
	std::set<ObstacleInterface *> obsSet = gEngine->getObstacles();
	//std::cout<<"now we got the set of obstacles"<<std::endl;
	if(!obsSet.empty()){
		//std::cout<<"now let's find the PolygonObstacle"<<std::endl;
		for(std::set<ObstacleInterface*>::iterator obst = obsSet.begin(); obst != obsSet.end(); obst ++){
			if(dynamic_cast<PolygonObstacle *>(*obst)){
				
				std::vector<Util::Vector> obsShape;
				(*obst)->returnVertices(obsShape);
				
				std::vector<Util::Vector> agentShape;
				float width  = this->_radius + _SocialForcesParams.sf_query_radius;
	       		agentShape.push_back(Util::Vector(_position.x - width, 0.0f, _position.z - width));
	       		agentShape.push_back(Util::Vector(_position.x + width, 0.0f, _position.z - width));
	       		agentShape.push_back(Util::Vector(_position.x + width, 0.0f, _position.z + width));
	       		agentShape.push_back(Util::Vector(_position.x - width, 0.0f, _position.z + width));
	    
	       		std::vector<Util::Vector> simplex;

	       		if(SteerLib::GJK_EPA::GJK(obsShape, agentShape, simplex)){ // polygon obstacle in range
	       	

	       			Util::Vector obs_normal;
					std::pair<Util::Point, Util::Point> line;
					std::pair<float, Util::Point> min_stuff;
					line = getLine(obsShape, this->position(), obs_normal);
					//std::cout<<position()<<" line "<<line.first<<" "<<line.second<<std::endl;
				    min_stuff = minimum_distance(line.first, line.second, this->position());
				    //std::cout<<"agent position, second "<<this->position()<<" "<<min_stuff.second<<std::endl;
				    //std::cout<<"normal is "<<obs_normal<<std::endl;

				    Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
				    obs_proximity_force = obs_proximity_force + obs_normal * _SocialForcesParams.sf_wall_a * 1 *
	                    exp(
	                            ( this->radius() - min_stuff.first)/_SocialForcesParams.sf_wall_b
	                       )
	                    * dt; 
	                //std::cout<<"min_stuff "<<min_stuff.first<<"radius "<<this->radius()<<std::endl;
	                //std::cout<<"Penetration "<<(*obst)->computePenetration(this->position(), this->radius())<<std::endl;

				   //if((*obst)->computePenetration(this->position(), this->radius())> 0.000001)
				   	if(min_stuff.first < this->radius()){
				   	//std::cout<<"Penetration "<<this->radius() - min_stuff.first<<std::endl;
				   	//if(true){

					   	obs_repulsion_force += obs_normal * (-min_stuff.first + radius()) * 1 *_SocialForcesParams.sf_body_force * dt;

						Util::Vector nij = normalize(position() - min_stuff.second);
						Util::Vector tij = Util::Vector(-nij.z, 0.0f, nij.x);
						float vji = (- this->velocity()) * tij;

						obs_sliding_force += ((*obst)->computePenetration(this->position(), this->radius()) * 1 *_SocialForcesParams.sf_sliding_friction_force * dt) * vji * tij;
				   }

	       		}
			}
		}
	}

	//std::cout<<"obs_repulsion_force "<<obs_repulsion_force<<std::endl;
	//std::cout<<"obs_siliding_force "<<obs_sliding_force<<std::endl;
	//std::cout<<"obs_proximity_force "<<obs_proximity_force<<std::endl;
	return obs_repulsion_force  +  obs_proximity_force;// + obs_sliding_force;
    //return Util::Vector(0,0,0);
}
std::pair<Util::Point, Util::Point> SocialForcesAgent::getLine(std::vector<Util::Vector> &shape, Util::Point p, Util::Vector &normal){ 
	//std::cout<<"getline is running"<<std::endl;
	Util::Vector position = p - Util::Point(0,0,0);
	float min_distance = (position - shape[0]).length();
	int v = 0;
	int size = shape.size();
	if(size > 1){
		//std::cout<<"the size of shape is "<<size<<std::endl;
		for(int i = 0; i < size; i ++){
			//suppose shape has n vertices, the cal normalized vector of every point
			float distance = (position - shape[i]).length();
			if(distance < min_distance){
				min_distance = distance;
				v = i;
			}
			
		}
		//std::cout<<shape[v]<<std::endl;
		#define next(x, size) (((x) + 1) % (size))
		#define prev(x, size) ((((x) - 1) + (size)) % (size))

		int next = next(v, size);
		int prev = prev(v, size); 
		Util::Vector dir;
		dir = normalize(normalize(shape[v] - shape[prev]) + normalize(shape[v] - shape[next]));
		Util::Vector agent = normalize(position - shape[v]);
		Util::Vector nextP = normalize(shape[next] - shape[v]);

		if(agent * nextP > dir * nextP){
			// point is on the next side of 
			Util::Point p1 = Util::Point(shape[v].x, shape[v].y, shape[v].z);
			Util::Point p2 = Util::Point(shape[next].x, shape[next].y, shape[next].z);
			
			//return std::make_pair(shape[v], shape[next]);

			normal = normalize(Util::Vector(-(shape[next] - shape[v]).z, 0.0f, (shape[next] - shape[v]).x));
			
			if(normal * (shape[next(next, size)] - shape[next]) > 0){
				normal.x = - normal.x;
				//normal.y = - normal.y;
				normal.z = -normal.z;
			}
			//std::cout<<"calculated normal "<<normal<<std::endl;
			return std::make_pair(p1, p2);
		}	
			// on the prev side
		normal =  normalize(Util::Vector(-(shape[v] - shape[prev]).z, 0.0f, (shape[v] - shape[prev]).x)); 
		if(normal * (shape[next(prev, size)] - shape[prev]) > 0){
				normal.x = - normal.x;
				//normal.y = - normal.y;
				normal.z = -normal.z;
			}
			//std::cout<<"calculated normal2 "<<normal<<std::endl;
		Util::Point p1 = Util::Point(shape[v].x, shape[v].y, shape[v].z);
		Util::Point p2 = Util::Point(shape[prev].x, shape[prev].y, shape[prev].z);
		return std::make_pair(p1, p2);

	}
	return std::make_pair(Util::Point(0,0,0), Util::Point(0,0,0)); 
}


void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this))
	{
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false))
		{
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
		Util::DrawLib::drawFlag( this->currentGoal().targetLocation, Color(0.5f,0.8f,0), 2);
		if ( this->currentGoal().goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL )
		{
			Color color(0.4,0.9,0.4);
			DrawLib::glColor(color);
			DrawLib::drawQuad(Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmin),
					Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmax),
					Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmax),
					Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmin));
		}
		int i;
		for (i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
			DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
		}
		// DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _radius, this->_color);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
					this->position()
					+
					(
						(
							dynamic_cast<AgentInterface*>(*neighbor)->position()
							-
							this->position()
						)
					/2), Util::Vector(1,0,0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while ( __oldPositions.size() > mostPoints )
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}

	for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}


