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
#include <pcl/registration/distances.h>
//************ LOCAL DEPENDANCIES ****************//
#include <aero_path_planning/planning_strategies/AStarCarrot.h>
//***********    NAMESPACES     ****************//

using namespace aero_path_planning;
using namespace aero_path_planning::astar_utilities;


//********************* HURISTIC/COST FUNCTIONS *********************//

double astar_utilities::ed_huristic(const tf::Point& point, const tf::Point& goal)
{
	return point.distance(goal);
}

double astar_utilities::pt_cost(const tf::Point& this_point, const tf::Point& last_point)
{
//	double pcost = 0;

//	switch(trait.getEnum())
//	{
//	case occupancy_grid::utilities::CellTrait::UNKOWN:
//	case occupancy_grid::utilities::CellTrait::FREE_LOW_COST:
//	case occupancy_grid::utilities::CellTrait::GOAL:
//		pcost = 1.0;
//		break;
//	case occupancy_grid::utilities::CellTrait::FREE_HIGH_COST:
//		pcost = 2.0;
//		break;
//	case occupancy_grid::utilities::CellTrait::TRAVERSED:
//		pcost = 1.25;
//		break;
//	default:
//		pcost = 1;
//		break;
//	}

	return astar_utilities::ed_huristic(this_point, last_point);
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

AStarNode::AStarNode(const tf::Point& location, const tf::Point& goal, const AStarNodePtr parent, cost_func cost, huristic_func huristic):
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

const tf::Point& AStarNode::getLocation() const
{
	return this->location_;
}

const AStarNode::AStarNodePtr& AStarNode::getParent() const
{
	return this->parent_;
}

bool AStarNode::sameLocation(const AStarNode& node) const
{
	return this->location_.distance(node.getLocation())==0;
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
	this->n_pxy_.setX(1);
	this->n_pxy_.setY(0);
	this->n_pxy_.setZ(0);

	this->n_xpy_.setX(0);
	this->n_xpy_.setY(1);
	this->n_xpy_.setZ(0);

	this->n_nxy_.setX(-1);
	this->n_nxy_.setY(0);
	this->n_nxy_.setZ(0);

	this->n_xny_.setX(0);
	this->n_xny_.setY(-1);
	this->n_xny_.setZ(0);

	this->n_dpxpy_.setX(1);
	this->n_dpxpy_.setY(1);
	this->n_dpxpy_.setZ(0);

	this->n_dnxny_.setX(-1);
	this->n_dnxny_.setY(-1);
	this->n_dnxny_.setZ(0);

	this->n_dnxpy_.setX(-1);
	this->n_dnxpy_.setY(1);
	this->n_dnxpy_.setZ(0);

	this->n_dpxny_.setX(1);
	this->n_dpxny_.setY(-1);
	this->n_dpxny_.setZ(0);
}

bool AStarCarrot::setCarrotDelta(double delta)
{
	this->delta_     = delta;
	this->has_delta_ = true;
	return this->has_delta_;
}

bool AStarCarrot::setSearchMap(occupancy_grid::MultiTraitOccupancyGridConstPtr map)
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

bool AStarCarrot::calcNeighbors(const tf::Point& point, std::vector<tf::Point>& neightbors) const
{

	tf::Point neighbor1(point+this->n_xpy_);


	tf::Point neighbor2(point + this->n_pxy_);


	tf::Point neighbor3(point + this->n_xny_);


	tf::Point neighbor4(point + this->n_nxy_);


	tf::Point neighbor5(point + this->n_dpxpy_);


	tf::Point neighbor6(point + this->n_dnxpy_);

	tf::Point neighbor7(point + this->n_dnxny_);


	tf::Point neighbor8(point + this->n_dpxny_);

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

void AStarCarrot::buildSolutionPath(const Node_t& goal_node, std::deque<geometry_msgs::Pose>& path) const
{
	geometry_msgs::Pose temp_pose;
	temp_pose.orientation.w = 1;
	tf::pointTFToMsg(goal_node.getLocation(), temp_pose.position);
	//Convert from grid cells to meters
	this->map_->gridCellToMeter(temp_pose.position.x, temp_pose.position.y, temp_pose.position.x, temp_pose.position.y);
	path.push_front(temp_pose);
	NodePtr_t path_node(goal_node.getParent());
	tf::Point lastPoint = goal_node.getLocation();

	while(path_node->getParent() != NodePtr_t())
	{
		if(path_node->getLocation().distance(lastPoint)>this->delta_)
		{
			tf::pointTFToMsg(path_node->getLocation(), temp_pose.position);
			//Convert from grid cells to meters
			this->map_->gridCellToMeter(temp_pose.position.x, temp_pose.position.y, temp_pose.position.x, temp_pose.position.y);
			path.push_front(temp_pose);
			lastPoint = path_node->getLocation();
		}
		path_node = path_node->getParent();
	}
	tf::pointTFToMsg(path_node->getLocation(), temp_pose.position);
	path.push_front(temp_pose);

//	for(int i=temp_path.size()-1; i>0; i--)
//	{
//		path.push_back(temp_path.at(i));
//	}

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

bool AStarCarrot::search(const geometry_msgs::Pose& start_point, const geometry_msgs::Pose& goal_point, ros::Duration& timeout, std::deque<geometry_msgs::Pose>& result_path)
{
	bool success    = false;

	if(this->canSearch())
	{
		ROS_INFO_STREAM("I'm searching from: "<<start_point<<", to: "<<goal_point);
		//ROS_INFO("I'm Searching!");
		//Set up the huristics
		cost_func     costf = boost::bind(&pt_cost, _1, _2);
		huristic_func hursf = boost::bind(&ed_huristic, _1, _2);

		//Create open/closed sets
		aero_path_planning::FitnessQueue<Node_t, NodePtr_t> open_set;
		boost::unordered_map<std::string, NodePtr_t>        closed_set;
		std::vector<tf::Point>                              neighbors;

		//Set timout checking conditions
		ros::Time start_time   = ros::Time::now();
		ros::Time current_time = start_time;

		//Set up the initial node
		tf::Point start;
		tf::Point goal;
		tf::pointMsgToTF(start_point.position, start);
		tf::pointMsgToTF(goal_point.position, goal);

		//Convert the start/goal points to grid-cells
		int tx;
		int ty;
		this->map_->meterToGridCell(start.x(), start.y(), tx, ty);
		start.setX(tx);
		start.setY(ty);
		this->map_->meterToGridCell(goal.x(), goal.y(), tx, ty);
		goal.setX(tx);
		goal.setY(ty);

		NodePtr_t start_node(new Node_t(start, goal, NodePtr_t(), costf, hursf));
		//ROS_INFO_STREAM("Start Node Set to: "<<(*start_node));
		open_set.push(*start_node, start_node);

		//Set up a node to use to check for the termination condition, and will contain the solution path head
		//upon search compleation
		Node_t    goal_node_dummy(goal, goal, NodePtr_t(), costf, hursf);
		//ROS_INFO_STREAM("Goal Node Set to: "<<goal_node_dummy);


		//ROS_INFO_STREAM("I'm Searching with "<<PRINT_POINT_S("Start Point", start_point)<<" and "<<PRINT_POINT_S("Goal Point", goal_point));

		bool timeout_c = false;
		bool os_empty  = false;
		while(!success&&!timeout_c&&!os_empty)
		{
			if(closed_set.size()%10000 == 0)
			{
				ROS_INFO_STREAM("I've expanded "<<closed_set.size()<<" nodes of"<<map_->getXSizeGrid()*map_->getYSizeGrid()<<" possible nodes");
			}
			os_empty  = open_set.empty();
			timeout_c = (current_time-start_time)>timeout;
			//Get the next node to expand
			NodePtr_t current_node = open_set.top();
			open_set.pop();

			//ROS_INFO_STREAM("I'm Expanding "<<current_node);

			std::stringstream node_rep;
			nodeLocationToStream(node_rep, current_node->getLocation());
			//ROS_INFO_STREAM("The Node Representation was:"<<node_rep.str());

			//Check to see if we hit the goal
			if(current_node->sameLocation(goal_node_dummy))
			{
				//ROS_INFO_STREAM("I Hit the Goal with "<<current_node);
				closed_set[node_rep.str()] = current_node;
				goal_node_dummy = *current_node;
				success         = true;
			}

			//Add the current node to the closed set

			if(closed_set.count(node_rep.str())==0)
			{
				//ROS_INFO_STREAM("I'm pushing "<<current_node<<"onto the closed set...");
				closed_set[node_rep.str()] = current_node;

				//Get the neighbors to the current node and add them to the open set
				neighbors.clear();
				this->calcNeighbors(current_node->getLocation(), neighbors);
				//ROS_INFO_STREAM("I got "<<neighbors.size()<<" neighbors for "<<current_node);

				for(unsigned int i=0; i<neighbors.size(); i++)
				{
					//Check to see if the neighbor is free
					if(!this->collision_checker_(neighbors.at(i), *this->map_))
					{
						NodePtr_t node(new Node_t(neighbors.at(i), goal, current_node, costf, hursf));
						std::stringstream neighbor_rep;
						nodeLocationToStream(neighbor_rep, node->getLocation());

						//Check to see if we've already expanded the neighbor node
						if(closed_set.count(neighbor_rep.str())==0 && !this->openSetContains(node, open_set))
						{
							//ROS_INFO_STREAM("I'm adding "<<node<<" to the open set");
							open_set.push(*node, node);
						}
					}
				}
			}
			else
			{
				//ROS_INFO_STREAM(current_node<<" was already in the closed set");
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
