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
#include <iostream>
#include <boost/unordered_map.hpp>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
using namespace aero_path_planning::astar_utilities;


//********************* HURISTIC/COST FUNCTIONS *********************//

double astar_utilities::ed_huristic(const aero_path_planning::Point& point, const aero_path_planning::Point& goal)
{
	return pcl::distances::l2(point.getVector4fMap(), goal.getVector4fMap());
}

double astar_utilities::pt_cost(const aero_path_planning::Point& this_point, const aero_path_planning::Point& last_point)
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

	return astar_utilities::ed_huristic(this_point, last_point)*pcost;
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
		this->g_ =  cost(location, parent->getLocation())+parent->getG();
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
	bool same = true;
	same &= this->location_.x == node.getLocation().x;
	same &= this->location_.y == node.getLocation().y;
	same &= this->location_.z == node.getLocation().z;

	return same;
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
	return this->getF() > rhs.getF();
}
bool       AStarNode::operator> (AStarNode const & rhs) const
{
	return this->getF() < rhs.getF();
}
bool       AStarNode::operator<= (AStarNode const & rhs) const
		{
	return this->getF() >= rhs.getF();
		}
bool       AStarNode::operator>= (AStarNode const & rhs) const
		{
	return this->getF() <= rhs.getF();
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
	this->setUpNeighborPoints();
}

AStarCarrot::AStarCarrot(const AStarCarrot& copy)
{
	*this = copy;
}

AStarCarrot::~AStarCarrot()
{

}

void AStarCarrot::setUpNeighborPoints()
{
	this->n_pxy_.x = 1;
	this->n_pxy_.y = 0;
	this->n_pxy_.z = 0;

	this->n_xpy_.x = 0;
	this->n_xpy_.y = 1;
	this->n_xpy_.z = 0;

	this->n_nxy_.x = -1;
	this->n_nxy_.y = 0;
	this->n_nxy_.z = 0;

	this->n_xny_.x = 0;
	this->n_xny_.y = -1;
	this->n_xny_.z = 0;

	this->n_dpxpy_.x = 1;
	this->n_dpxpy_.y = 1;
	this->n_dpxpy_.z = 0;

	this->n_dnxny_.x = -1;
	this->n_dnxny_.y = -1;
	this->n_dnxny_.z = 0;

	this->n_dnxpy_.x = -1;
	this->n_dnxpy_.y = 1;
	this->n_dnxpy_.z = 0;

	this->n_dpxny_.x = 1;
	this->n_dpxny_.y = -1;
	this->n_dpxny_.z = 0;
}

bool AStarCarrot::setCarrotDelta(double delta)
{
	this->delta_     = delta;
	this->has_delta_ = true;
	return this->has_delta_;
}

bool AStarCarrot::setSearchMap(const aero_path_planning::OccupancyGrid& map)
{
	this->map_     = map;
	this->has_map_ = true;
	return this->has_map_;
}

bool AStarCarrot::setCollision(collision_func_& collision_checker)
{
	this->collision_checker_ = collision_checker;
	this->has_coll_          = true;
	return this->has_coll_;
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

bool AStarCarrot::calcNeighbors(const Point& point, std::vector<Point>& neightbors) const
{

	Point neighbor1;
	neighbor1.getVector4fMap() = point.getVector4fMap() + this->n_xpy_.getVector4fMap();


	Point neighbor2;
	neighbor2.getVector4fMap() = point.getVector4fMap() + this->n_pxy_.getVector4fMap();


	Point neighbor3;
	neighbor3.getVector4fMap() = point.getVector4fMap() + this->n_xny_.getVector4fMap();


	Point neighbor4;
	neighbor4.getVector4fMap() = point.getVector4fMap() + this->n_nxy_.getVector4fMap();


	Point neighbor5;
	neighbor5.getVector4fMap() = point.getVector4fMap() + this->n_dpxpy_.getVector4fMap();


	Point neighbor6;
	neighbor6.getVector4fMap() = point.getVector4fMap() + this->n_dnxpy_.getVector4fMap();

	Point neighbor7;
	neighbor7.getVector4fMap() = point.getVector4fMap() + this->n_dnxny_.getVector4fMap();


	Point neighbor8;
	neighbor8.getVector4fMap() = point.getVector4fMap() + this->n_dpxny_.getVector4fMap();

	neightbors.push_back(neighbor1);
	neightbors.push_back(neighbor2);
	neightbors.push_back(neighbor3);
	neightbors.push_back(neighbor4);
	neightbors.push_back(neighbor5);
	neightbors.push_back(neighbor6);
	neightbors.push_back(neighbor7);
	neightbors.push_back(neighbor8);


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
	temp_path.push_back(path_node->getLocation());

	for(int i=temp_path.size()-1; i>0; i--)
	{
		path.push(temp_path.at(i));
	}

}

bool AStarCarrot::canSearch() const
{
	return this->has_coll_&&this->has_map_&&this->has_delta_;
}

bool AStarCarrot::openSetContains(const NodePtr_t& node, const aero_path_planning::FitnessQueue<Node_t, NodePtr_t>& open_set) const
{
	typedef aero_path_planning::FitnessQueue<Node_t, NodePtr_t>::const_iterator citr_t;
	bool contains = false;

//	ROS_INFO_STREAM("I'm checking the open set for "<<node);

	for(citr_t itr = open_set.c_begin(); itr < open_set.c_end(); itr++)
	{
//		ROS_INFO_STREAM("Comparing "<<(*itr)->second);
		if((*itr)->second->sameLocation(*node))
		{
//			ROS_INFO_STREAM(node<<" was in the open set!");
			contains = true;
			break;
		}
	}

	return contains;
}

bool AStarCarrot::search(const Point& start_point, const Point& goal_point, ros::Duration& timeout, std::queue<Point>& result_path)
{
	bool success    = false;

	if(this->canSearch())
	{
		ROS_INFO("I'm Searching!");
		//Set up the huristics
		cost_func     costf = boost::bind(&pt_cost, _1, _2);
		huristic_func hursf = boost::bind(&ed_huristic, _1, _2);

		//Create open/closed sets
		aero_path_planning::FitnessQueue<Node_t, NodePtr_t> open_set;
		boost::unordered_map<std::string, NodePtr_t>        closed_set;
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


		ROS_INFO_STREAM("I'm Searching with "<<PRINT_POINT_S("Start Point", start_point)<<" and "<<PRINT_POINT_S("Goal Point", goal_point));

		bool timeout_c = false;
		bool os_empty  = false;
		while(!success&&!timeout_c&&!os_empty)
		{
			ROS_INFO_STREAM("I've expanded "<<closed_set.size()<<" nodes of"<<map_.size()<<" possible nodes");
			os_empty  = open_set.empty();
			timeout_c = (current_time-start_time)>timeout;
			//Get the next node to expand
			NodePtr_t current_node = open_set.top();
			open_set.pop();

			ROS_INFO_STREAM("I'm Expanding "<<current_node<<"...");

			std::stringstream node_rep;
			node_rep<<PRINT_POINT_S("",current_node->getLocation());

			//Check to see if we hit the goal
			if(current_node->sameLocation(goal_node_dummy))
			{
				ROS_INFO_STREAM("I Hit the "<<PRINT_POINT_S("Goal: ", goal_node_dummy.getLocation())<<" with "<<current_node);
				closed_set[node_rep.str()] = current_node;
				goal_node_dummy = *current_node;
				success         = true;
			}

			//Add the current node to the closed set

			if(closed_set.count(node_rep.str())==0)
			{
				ROS_INFO_STREAM("I'm pushing "<<current_node<<"onto the closed set...");
				closed_set[node_rep.str()] = current_node;

				//Get the neighbors to the current node and add them to the open set
				neighbors.clear();
				this->calcNeighbors(current_node->getLocation(), neighbors);
				ROS_INFO_STREAM("I got "<<neighbors.size()<<" neighbors for "<<current_node);

				for(unsigned int i=0; i<neighbors.size(); i++)
				{
					//Check to see if the neighbor is free
					if(!this->collision_checker_(neighbors.at(i), this->map_))
					{
						NodePtr_t node(new Node_t(neighbors.at(i), goal_point, current_node, costf, hursf));
						std::stringstream neighbor_rep;
						neighbor_rep<<PRINT_POINT_S("",node->getLocation());

						//Check to see if we've already expanded the neighbor node
						if(closed_set.count(neighbor_rep.str())==0 && !this->openSetContains(node, open_set))
						{
							ROS_INFO_STREAM("I'm adding "<<node<<" to the open set");
							open_set.push(*node, node);
						}
					}
				}
			}
			else
			{
				ROS_INFO_STREAM(current_node<<" was already in the closed set");
			}

//			std::string pause;
	//		std::cin>>pause;

		}
		//If we timed out, say so
		if(timeout_c)
		{
			ROS_WARN("I Could Not Find A Solution In Time");
		}
		//If we ran out of nodes without finding a solution, say so
		if(os_empty)
		{
			ROS_WARN("I expanded every node in the grid without a solution");
		}

		//If we got a path, stuff the solution vector
		if(success)
		{
			ROS_INFO_STREAM("I Got A Solution, Generating Solution Path");
			this->buildSolutionPath(goal_node_dummy, result_path);
		}
	}
	else
	{
		ROS_ERROR("Cannot Perform Search On Uninitialized Planner!");
	}

	return success;
}
