/**
 * @file   AStartCarrot.cpp
 *
 * @date   Mar 29, 2013
 * @author Adam Panzica
 * @brief  Implementation of the A* algorithm as a CarrotPathFinder
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <pcl/registration/distances.h>
#include <list>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
using namespace aero_path_planning::astar_utilities;


//********************* HURISTIC/COST FUNCTIONS *********************//

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

//********************* ASTARNODE IMPLEMENTATION *********************//

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

const AStarNode::AStarNodePtr& AStarNode::getParent() const
{
	return this->parent_;
}

bool AStarNode::sameLocation(const AStarNode& node) const
{
	return this->location_.getVector4fMap() == node.getLocation().getVector4fMap();
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

//********************* ASTARPLANNER *********************//
AStarCarrot::AStarCarrot():
		has_delta_(false),
		has_map_(false),
		has_coll_(false)
{

}

AStarCarrot::AStarCarrot(const AStarCarrot& copy)
{
	*this = copy;
}

AStarCarrot::~AStarCarrot()
{

}

bool AStarCarrot::setCarrotDelta(double delta)
{
	this->delta_     = delta;
	this->has_delta_ = true;
	return true;
}

bool AStarCarrot::setSearchMap(const aero_path_planning::OccupancyGrid& map)
{
	this->map_     = map;
	this->has_map_ = true;
	return true;
}

bool AStarCarrot::setCollision(collision_func_& collision_checker)
{
	this->collision_checker_ = collision_checker;
	this->has_coll_          = true;
	return true;
}

bool AStarCarrot::allowsPartialPath()
{
	return false;
}

bool AStarCarrot::getPlanningType(std::string& type) const
{
	type = "A* Carrot";
	return true;
}

AStarCarrot& AStarCarrot::operator =(const AStarCarrot& copy)
{
	this->collision_checker_ = copy.collision_checker_;
	this->delta_             = copy.delta_;
	this->has_coll_          = copy.has_coll_;
	this->has_delta_         = copy.has_delta_;
	this->has_map_           = copy.has_map_;
	this->map_               = copy.map_;
	return *this;
}

bool AStarCarrot::calcNeighbors(const Point& point, std::vector<Point> neightbors) const
{
	return true;
}

void AStarCarrot::buildSolutionPath(const Node_t& goal_node, std::queue<Point>& path) const
{
	std::vector<Point> temp_path;
	temp_path.push_back(goal_node.getLocation());
	NodePtr_t path_node(goal_node.getParent());

	while(path_node->getParent() != NodePtr_t())
	{
		temp_path.push_back(path_node->getLocation());
		path_node = path_node->getParent();
	}

	BOOST_FOREACH(std::vector<Point>::value_type point, temp_path)
	{
		path.push(point);
	}

}

bool AStarCarrot::search(const Point& start_point, const Point& goal_point, ros::Duration& timeout, std::queue<Point>& result_path)
{
	bool can_search = this->has_coll_&&this->has_delta_&&this->has_map_;
	bool success = false;

	if(can_search)
	{
		//Set up the huristics
		cost_func     costf = boost::bind(&pt_cost, _1, _2);
		huristic_func hursf = boost::bind(&ed_huristic, _1, _2);

		//Create open/closed sets
		aero_path_planning::FitnessQueue<Node_t, NodePtr_t> open_set;
		std::list<NodePtr_t>                                closed_set;
		std::vector<Point>                                  neighbors;

		//Set timout checking conditions
		ros::Time start_time   = ros::Time::now();
		ros::Time current_time = start_time;

		//Set up the initial node
		NodePtr_t start_node(new Node_t(start_point, goal_point, NodePtr_t(), costf, hursf));
		open_set.push(*start_node, start_node);

		//Set up a node to use to check for the termination condition, and will contain the solution path head
		//upon search compleation
		Node_t    goal_node_dummy(goal_point, goal_point, NodePtr_t(), costf, hursf);

		while(!success&&(current_time-start_time)<timeout&&open_set.size()>0)
		{
			//Get the next node to expand
			NodePtr_t current_node = open_set.top();
			open_set.pop();

			//Check to see if we hit the goal
			if(current_node->sameLocation(goal_node_dummy))
			{
				closed_set.push_back(current_node);
				goal_node_dummy = *current_node;
				success         = true;
			}

			//Add the current node to the closed set
			closed_set.push_back(current_node);

			//Get the neighbors to the current node and add them to the open set
			neighbors.clear();
			this->calcNeighbors(current_node->getLocation(), neighbors);

			for(unsigned int i=0; i<neighbors.size(); i++)
			{
				if(!this->collision_checker_(neighbors.at(i), this->map_))
				{
					NodePtr_t node(new Node_t(neighbors.at(i), goal_point, current_node, costf, hursf));
					open_set.push(*node, node);
				}
			}

		}

		//If we got a path, stuff the solution vector
		if(success)
		{
			this->buildSolutionPath(goal_node_dummy, result_path);
		}
	}

	return success;
}
