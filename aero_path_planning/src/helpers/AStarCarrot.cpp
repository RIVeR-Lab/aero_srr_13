/**
 * @file   AStartCarrot.cpp
 *
 * @date   Mar 29, 2013
 * @author Adam Panzica
 * @brief  Implementation of the A* algorithm as a CarrotPathFinder
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <pcl/registration/distances.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
using namespace aero_path_planning::astar_utilities;

double ed_huristic(const aero_path_planning::Point& point, const aero_path_planning::Point& goal)
{
	return pcl::distances::l2(point.getVector4fMap(), goal.getVector4fMap());
}

double pt_cost(const aero_path_planning::Point& this_point, const aero_path_planning::Point& last_point)
{
	double pcost = 0;

	switch(this_point.rgba)
	{
	case aero_path_planning::UNKNOWN:
	case aero_path_planning::FREE_LOW_COST:
	case aero_path_planning::GOAL:
		pcost = 1.0;
		break;
	case aero_path_planning::FREE_HIGH_COST:
		pcost = 2.0;
		break;
	case aero_path_planning::TRAVERSED:
		pcost = 1.25;
		break;
	default:
		pcost = 1;
		break;
	}

	return ed_huristic(this_point, last_point)*pcost;
}

AStarNode::AStarNode():
		g_(0),
		h_(0),
		f_(std::numeric_limits<double>::infinity())
{

}

AStarNode::AStarNode(const AStarNode& copy)
{
	*this = copy;
}

AStarNode::AStarNode(const AStarNodePtr& copy)
{
	*this = *copy;
}

AStarNode::AStarNode(const Point& location, const Point& goal, const AStarNodePtr parent, cost_func cost, huristic_func huristic):
		location_(location),
		parent_(parent),
		h_(huristic(location, goal))
{
	if(parent != AStarNodePtr())
	{
		this->g_ = cost(location, parent->getLocation())+parent->getG();
	}
	else
	{
		this->g_ = 0;
	}
	this->f_ = this->h_+this->g_;
}

double AStarNode::getG() const
{
	return this->g_;
}

double AStarNode::getH() const
{
	return this->h_;
}

double AStarNode::getF() const
{
	return this->f_;
}

const aero_path_planning::Point& AStarNode::getLocation() const
{
	return this->location_;
}

AStarNode& AStarNode::operator= (AStarNode const & rhs)
{
	this->location_ = rhs.location_;
	this->parent_   = rhs.parent_;
	this->f_        = rhs.f_;
	this->g_        = rhs.g_;
	this->h_        = rhs.h_;
	return *this;
}

bool       AStarNode::operator< (AStarNode const & rhs) const
{
	return this->getF() < rhs.getF();
}
bool       AStarNode::operator> (AStarNode const & rhs) const
{
	return this->getF() > rhs.getF();
}
bool       AStarNode::operator<= (AStarNode const & rhs) const
{
	return this->getF() <= rhs.getF();
}
bool       AStarNode::operator>= (AStarNode const & rhs) const
{
	return this->getF() >= rhs.getF();
}
bool       AStarNode::operator==(AStarNode const & rhs) const
{
	return this->getF() == rhs.getF();
}





