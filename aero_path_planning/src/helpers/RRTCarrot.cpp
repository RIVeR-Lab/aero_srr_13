/**
 * @file RRTCarrot.cpp
 *
 * @date   Feb 19, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//

//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/RRTCarrot.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

RRTCarrot::RRTCarrot(const aero_path_planning::RRTCarrot& copy):
		initialized_(copy.initialized_),
		has_delta_(copy.has_delta_),
		has_coll_(copy.has_coll_),
		has_map_(copy.has_map_),
		delta_(copy.delta_),
		map_(copy.map_),
		collision_checker_(copy.collision_checker_)
{

}

RRTCarrot::RRTCarrot():
		initialized_(false),
		has_delta_(false),
		has_coll_(false),
		has_map_(false),
		delta_(0)
{

}

RRTCarrot::~RRTCarrot()
{

}

void RRTCarrot::isInialized()
{
	this->initialized_ = this->has_coll_&&this->has_delta_&&this->has_map_;
}

bool RRTCarrot::setCollision(collision_func_& collision_checker)
{
	this->collision_checker_ = collision_checker;
	this->has_coll_;
	this->isInialized();
	return true;
}

bool RRTCarrot::setSearchMap(const aero_path_planning::OccupancyGrid& map)
{
	this->map_ = map;
	this->has_map_;
	this->isInialized();
	return true;
}

bool RRTCarrot::setCarrotDelta(int delta)
{
	this->delta_ = delta;
	this->has_delta_;
	this->isInialized();
	return true;
}

bool RRTCarrot::search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, std::queue& result_path)
{
	if(this->isInialized())
	{
		return true;
	}
	else
	{
		ROS_ERROR("Cannot Perform Path Planning On Uninitialized Planner!");
		return false;
	}
}

bool RRTCarrot::getType(std::string& type) const
{
	type = "RRT Carrot Planner";
	return true;
}

RRTCarrot& RRTCarrot::operator=(RRTCarrot const &copy)
{
	this->initialized_ = copy.initialized_;
	this->has_delta_   = copy.has_delta_;
	this->has_coll_    = copy.has_coll_;
	this->has_map_     = copy.has_map_;
	this->delta_       = copy.delta_;
	this->map_         = copy.map_;
	this->collision_checker_ = copy.collision_checker_;
	return *this;
}
