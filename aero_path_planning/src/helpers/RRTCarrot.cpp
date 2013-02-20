/**
 * @file RRTCarrot.cpp
 *
 * @date   Feb 19, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<boost/foreach.hpp>
#include<boost/random.hpp>
#include<boost/random/normal_distribution.hpp>
#include<pcl/common/norms.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/RRTCarrot.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

//*******************RRTCarrotTree*****************************//
RRTCarrotTree::RRTCarrotTree()
{

}

RRTCarrotTree::RRTCarrotTree(const RRTCarrotTree& copy):
				nodes_(copy.nodes_)
{

}

RRTCarrotTree::RRTCarrotTree(const RRTCarrotTree* copy):
			nodes_(copy->nodes_)
{

}

RRTCarrotTree::RRTCarrotTree(int size):
				nodes_(size)
{

}

bool RRTCarrotTree::addNode(const RRTNode& node)
{
	this->nodes_.push_back(node);
	return true;
}

RRTNode* RRTCarrotTree::findNearestNeighbor(const RRTNode* to_node)const
{
	double best_distance = std::numeric_limits<double>::infinity();
	RRTNode* nearest;
	BOOST_FOREACH(node_deque::value_type item, this->nodes_)
	{
		double dist = std::abs(pcl::distances::l2(item.location->getVector4fMap(), to_node->location->getVector4fMap()));
		if(dist<best_distance)
		{
			nearest = &item;
		}
	}
	return nearest;
}

RRTCarrotTree::size_type RRTCarrotTree::size() const
{
	return this->nodes_.size();
}

void RRTCarrotTree::flushTree()
{
	this->nodes_.clear();
}

RRTCarrotTree& RRTCarrotTree::operator=(RRTCarrotTree const &copy)
{
	this->nodes_ = copy.nodes_;
	return *this;
}

//*********************RRTCarrot*******************************//

RRTCarrot::RRTCarrot(const aero_path_planning::RRTCarrot& copy):
		step_size_(copy.step_size_),
		initialized_(copy.initialized_),
		has_delta_(copy.has_delta_),
		has_coll_(copy.has_coll_),
		has_map_(copy.has_map_),
		delta_(copy.delta_),
		map_(copy.map_),
		collision_checker_(copy.collision_checker_),
		start_tree_(NULL),
		goal_tree_(NULL),
		rand_gen_(NULL)
{

	this->start_tree_  = new RRTCarrotTree(copy.start_tree_);
	this->goal_tree_   = new RRTCarrotTree(copy.goal_tree_);
	this->randInit();
}

RRTCarrot::RRTCarrot():
		step_size_(0),
		initialized_(false),
		has_delta_(false),
		has_coll_(false),
		has_map_(false),
		delta_(0),
		start_tree_(NULL),
		goal_tree_(NULL),
		rand_gen_(NULL)
{
	this->randInit();
}

RRTCarrot::RRTCarrot(double step_size):
		step_size_(step_size),
		initialized_(false),
		has_delta_(false),
		has_coll_(false),
		has_map_(false),
		delta_(0),
		start_tree_(NULL),
		goal_tree_(NULL),
		rand_gen_(NULL)
{
	this->randInit();
}

RRTCarrot::~RRTCarrot()
{
	if(this->start_tree_!=NULL) delete this->start_tree_;
	if(this->goal_tree_ !=NULL) delete this->goal_tree_;
	if(this->rand_gen_  !=NULL) delete this->rand_gen_;
}

void RRTCarrot::randInit()
{
	boost::mt19937 RNG;
	boost::normal_distribution<> dist(0, 1);
	this->rand_gen_  = new boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(RNG, dist);
	this->rand_gen_->engine().seed();
	this->rand_gen_->distribution().reset();
}

void RRTCarrot::isInialized()
{
	this->initialized_ = this->has_coll_&&this->has_delta_&&this->has_map_;
}

bool RRTCarrot::setCollision(collision_func_& collision_checker)
{
	this->collision_checker_ = collision_checker;
	this->has_coll_ = true;
	this->isInialized();
	return true;
}

bool RRTCarrot::setSearchMap(const aero_path_planning::OccupancyGrid& map)
{
	this->map_ = map;
	this->has_map_ = true;
	//Since assiging a new map would invalidate any trees we've built, delete them if they exist
	if(this->start_tree_!=NULL)
	{
		delete this->start_tree_;
	}
	this->start_tree_ = new RRTCarrotTree(this->map_.size()/100);
	if(this->goal_tree_!=NULL)
	{
		delete this->goal_tree_;
	}
	this->goal_tree_  = new RRTCarrotTree(this->map_.size()/100);
	this->isInialized();
	return true;
}

bool RRTCarrot::setCarrotDelta(double delta)
{
	this->delta_ = delta;
	this->has_delta_ = true;
	this->isInialized();
	return true;
}

bool RRTCarrot::search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, std::queue<aero_path_planning::Point*>& result_path)
{
	if(this->initialized_)
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
	if(this->start_tree_!=NULL&&this->start_tree_!=copy.start_tree_)
	{
		delete this->start_tree_;
		this->start_tree_  = new RRTCarrotTree(copy.start_tree_);
	}
	else if(this->start_tree_==NULL)
	{
		this->start_tree_  = new RRTCarrotTree(copy.start_tree_);
	}
	if(this->goal_tree_!=NULL&&this->goal_tree_!=copy.goal_tree_)
	{
		delete this->goal_tree_;
		this->goal_tree_   = new RRTCarrotTree(copy.goal_tree_);
	}
	else if(this->goal_tree_==NULL)
	{
		this->goal_tree_   = new RRTCarrotTree(copy.goal_tree_);
	}
	this->initialized_ = copy.initialized_;
	this->has_delta_   = copy.has_delta_;
	this->has_coll_    = copy.has_coll_;
	this->has_map_     = copy.has_map_;
	this->delta_       = copy.delta_;
	this->map_         = copy.map_;
	this->collision_checker_ = copy.collision_checker_;

	return *this;
}
