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
#include<pcl/common/distances.h>
#include<pcl/common/norms.h>
//*****************LOCAL DEPENDANCIES**************************//
#include<aero_path_planning/RRTCarrot.h>
//**********************NAMESPACES*****************************//


using namespace aero_path_planning;

//*******************RRTCarrotTree*****************************//

RRTCarrotTree::RRTCarrotTree(const RRTCarrotTree& copy):
						nodes_(copy.nodes_)
{

}

RRTCarrotTree::RRTCarrotTree(const RRTCarrotTree* copy):
					nodes_(copy->nodes_)
{

}

RRTCarrotTree::RRTCarrotTree():
						nodes_()
{

}

RRTCarrotTree::~RRTCarrotTree()
{
	this->flushTree();
};

bool RRTCarrotTree::addNode(const node_ptr_t node)
{
	this->nodes_.push_back(node);
	return true;
}

const node_ptr_t& RRTCarrotTree::findNearestNeighbor(const node_ptr_t to_node)const
{
	double best_distance = std::numeric_limits<double>::max();
	double dist;
	const node_ptr_t* nearest  = NULL;
	for(node_deque::const_iterator node_itr = this->nodes_.begin(); node_itr<this->nodes_.end(); node_itr++)
	{
		dist = std::abs(pcl::distances::l2(to_node->location_.getVector4fMap(), (*node_itr)->location_.getVector4fMap()));
		if(dist<best_distance)
		{
			best_distance =  dist;
			nearest       = &(*node_itr);
		}
	}
	if(nearest!=NULL) return *nearest;
	else              return this->empty_node_;
}

const node_ptr_t& RRTCarrotTree::getLeafNode()
{
	if(this->nodes_.size()>0)
	{
		return this->nodes_.back();
	}
	else
	{
		return this->empty_node_;
	}
}

const node_ptr_t& RRTCarrotTree::getRootNode()
{
	if(this->nodes_.size()>0)
	{
		return this->nodes_.front();
	}
	else
	{
		return this->empty_node_;
	}
}

RRTCarrotTree::size_type RRTCarrotTree::size() const
{
	return this->nodes_.size();
}

void RRTCarrotTree::flushTree()
{
	BOOST_FOREACH(node_deque::value_type node, this->nodes_)
	{
		node.reset();
	}
	this->nodes_.clear();
}

void RRTCarrotTree::visualizeTree(sensor_msgs::Image& image, int x_height, int y_height)
{
	aero_path_planning::PointCloud treeCloud(x_height, y_height);
	BOOST_FOREACH(node_deque::value_type item, this->nodes_)
	{
		if(item->parent_!= NULL)
		{
			castLine(item->parent_->location_, item->location_, aero_path_planning::TENTACLE, treeCloud);
		}
	}
	pcl::toROSMsg(treeCloud, image);
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
	if(copy.start_tree_!=NULL)
	{
		this->start_tree_  = new RRTCarrotTree(*copy.start_tree_);
	}
	if(copy.goal_tree_!=NULL)
	{
		this->goal_tree_   = new RRTCarrotTree(*copy.goal_tree_);
	}
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
	this->rand_gen_->engine().seed(ros::Time::now().toSec());
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
	this->start_tree_ = new RRTCarrotTree();
	if(this->goal_tree_!=NULL)
	{
		delete this->goal_tree_;
	}
	this->goal_tree_  = new RRTCarrotTree();
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

bool RRTCarrot::sample(node_ptr_t node)
{
	int x, y, z;
	this->genLoc(&x, &y, &z);
	node->location_.x = x;
	node->location_.y = y;
	node->location_.z = z;
	node->location_.getVector4fMap() = node->location_.getVector4fMap() - this->map_.getOriginPoint().getVector4fMap();
	return true;
}

void RRTCarrot::genLoc(int* x, int* y, int* z)
{
	*x = this->map_.getXSize()*(*this->rand_gen_)();
	if(*x<0)
	{
		*x=0;
	}
	else if(*x>=this->map_.getXSize())
	{
		*x=this->map_.getXSize()-1;
	}
	*y = this->map_.getYSize()*(*this->rand_gen_)();
	if(*y<0)
	{
		*y=0;
	}
	else if(*y>=this->map_.getYSize())
	{
		*y=this->map_.getYSize()-1;
	}
	*z = this->map_.getZSize()*(*this->rand_gen_)();
	if(*z<0)
	{
		*z=0;
	}
	else if(*z>=this->map_.getZSize())
	{
		*z=this->map_.getZSize()-1;
	}
}

bool RRTCarrot::connect(const node_ptr_t q_rand, node_ptr_t tree_node, RRTCarrotTree* tree)
{
	//If the node is on the tree already, we already searched it, don't add it
	if(q_rand->location_.getVector4fMap()==tree_node->location_.getVector4fMap()) return false;

	ROS_INFO_STREAM("Attempting to connect ("<<tree_node->location_.x<<","<<tree_node->location_.y<<
			") to ("<<q_rand->location_.x<<","<<q_rand->location_.y<<")");

	//Set up the connection properties
	Eigen::Vector4f step_vector;
	node_ptr_t      last_node(tree_node);
	double dist     = std::abs(pcl::distances::l2(last_node->location_.getVector4fMap(),q_rand->location_.getVector4fMap()));
	this->generateStepVector(tree_node->location_, q_rand->location_, step_vector);

	//While we're less than one step-size away from q_rand, step nodes forward
	while(dist>this->step_size_)
	{
		node_ptr_t next_node(new RRTNode());

		//Make a step and see if it's in collision
		if(this->step(last_node, step_vector, next_node))
		{
			//If it wasn't, see how close we've gotten to q_rand and add the new node the tree
			dist               = std::abs(pcl::distances::l2(next_node->location_.getVector4fMap(),q_rand->location_.getVector4fMap()));
			next_node->parent_ = tree->getLeafNode();
			tree->addNode(next_node);
			//Have to do this to ensure pointer changes
			last_node = next_node;
		}
		else
		{
			//If it was in collision, we're done
			break;
		}

	}
	//If we got within a step size, we sucessfully connected the nodes, add q_rand to the tree
	if(dist<=this->step_size_)
	{
		if(last_node->location_.getVector4fMap()!=q_rand->location_.getVector4fMap())
		{
			node_ptr_t q_node(new RRTNode());
			q_node->parent_  = last_node;
			q_node->location_ = q_rand->location_;
			tree->addNode(q_node);
		}
		return true;
	}
	//Otherwise we didn't fully conect, return false
	return false;
}

bool RRTCarrot::step(node_ptr_t last_node, const Eigen::Vector4f& step_vector, node_ptr_t next_node)
{
	Point next_point;
	next_point.getVector4fMap() = last_node->location_.getVector4fMap()+step_vector;
	next_point.x     = std::floor(next_point.x);
	next_point.y     = std::floor(next_point.y);
	next_point.z     = std::floor(next_point.z);

	if(!this->collision_checker_(next_node->location_, this->map_))
	{
		next_node->parent_ = last_node;
		ROS_INFO_STREAM("I Stepped ("<<last_node->location_.x<<","<<last_node->location_.y<<
				") to ("<<next_node->location_.x<<","<<next_node->location_.y<<") and set set next_node's parent to "<<next_node->parent_);
		return true;
	}
	return false;
}

bool RRTCarrot::generateStepVector(const Point& from_point, const Point& to_point, Eigen::Vector4f& vector)
{
	vector       = to_point.getVector4fMap()-from_point.getVector4fMap();
	double scale = this->step_size_/vector.norm();
	vector       = vector*scale;
	return true;
}

bool RRTCarrot::allowsPartialPath()
{
	return true;
}

bool RRTCarrot::search(const aero_path_planning::Point& start_point, const aero_path_planning::Point& goal_point, ros::Duration& timeout, std::queue<aero_path_planning::Point>& result_path)
{
	if(this->initialized_)
	{
		std::string search_type;
		this->getPlanningType(search_type);
		std::stringstream start_str;
		start_str<<"("<<start_point.x<<","<<start_point.y<<")";
		std::stringstream goal_str;
		goal_str<<"("<<goal_point.x<<","<<goal_point.y<<")";
		ROS_INFO_STREAM("I'm Performing a "<<search_type<<" Search from start_point <"<<start_str.str()<<"> to <"<<goal_str.str()<<">");

		//Get the current time for timeout checks
		ros::Time start_time = ros::Time::now();

		//Build the initial nodes, add to trees
		bool sucess = false;
		bool time_out = false;

		node_ptr_t start_node (new RRTNode());
		start_node->parent_ = node_ptr_t();
		start_node->location_= start_point;
		node_ptr_t goal_node(new RRTNode());
		goal_node->parent_  = node_ptr_t();
		goal_node->location_ = goal_point;

		this->start_tree_->flushTree();
		this->start_tree_->addNode(start_node);
		this->goal_tree_->flushTree();
		this->goal_tree_->addNode(goal_node);

		bool from_start = true; //Flag to signal if we're sampling randomly from start_tree or goal_tree
		RRTCarrotTree* sample_tree = this->start_tree_;
		RRTCarrotTree* connect_tree= this->goal_tree_;

		ROS_INFO_STREAM("Tree Set Up, Beginning Search!");

		while(!sucess&&!time_out&&ros::ok())
		{
			//Sample a new random point on the grid
			node_ptr_t q_rand(new RRTNode());
			q_rand->parent_ = node_ptr_t();
			this->sample(q_rand);

			//debug
			std::stringstream node_str;
			node_str<<"("<<q_rand->location_.x<<","<<q_rand->location_.y<<")";
			ROS_INFO_STREAM("Sampling Point "<<node_str.str());


			//Check to see if the point is in collision
			if(!this->collision_checker_(q_rand->location_, this->map_))
			{
				//debug
				ROS_INFO_STREAM("Got a valid sample point!");

				//Swap which tree we're sampling from and witch tree we're connecting to as needed
				if(!from_start)
				{
					std::swap(sample_tree, connect_tree);
				}

				//Connect the nearest neighbor on the sampling tree to the sampled point
				this->connect(q_rand, sample_tree->findNearestNeighbor(q_rand), sample_tree);
				//Attempt to connect the sample tree to the connect tree, if sucessfull we're done as we have a path
				sucess = this->connect(sample_tree->getLeafNode(), connect_tree->getLeafNode(), connect_tree);
				from_start = !from_start;
			}
			else
			{
				ROS_INFO("Sampled Point Was In Collision");
			}
			time_out = (ros::Time::now()-start_time)>timeout;
			//debug
			ROS_INFO_STREAM("Tree Sizes Thus Far: start_tree="<<this->start_tree_->size()<<" nodes, goal_tree= "<<this->goal_tree_->size()<<" nodes");

		}
		ROS_INFO_STREAM("Connected The Trees or ROS Quit, Building Path If ROS Running...");
		if(ros::ok())
		{
			//Merge the paths together. As the last nodes added to each tree will have been the connect nodes, we use those as entires to merge
			if(!from_start)
			{
				//Make sure we're building a path from the start to the goal
				std::swap(sample_tree, connect_tree);
			}

			//Build the final complete path
			node_ptr_t next_node_on_path;
			//If we didn't timeout, merge the trees
			if(!time_out)
			{
				ROS_INFO("Merging Paths...");
				if(!this->mergePath(sample_tree->getLeafNode(), connect_tree->getLeafNode())) ROS_ERROR("Something Went Wrong Merging Paths!");
				next_node_on_path = connect_tree->getRootNode();
			}
			//Otherwise build the path that got closest to goal
			else
			{
				ROS_INFO("Building Incomplete Path...");
				next_node_on_path = sample_tree->findNearestNeighbor(goal_node);
			}

			while(next_node_on_path != node_ptr_t())
			{
				ROS_INFO_STREAM("Adding Node At ("<<next_node_on_path->location_.x<<","<<next_node_on_path->location_.y<<")");
				next_node_on_path->location_.rgba = aero_path_planning::GOAL;
				result_path.push(next_node_on_path->location_);
				next_node_on_path = next_node_on_path->parent_;
			}
		}
		else return false;
		//Built the path, we're done
		return sucess;
	}
	else
	{
		ROS_ERROR("Cannot Perform Path Planning On Uninitialized Planner!");
		return false;
	}
}

bool RRTCarrot::getPlanningType(std::string& type) const
{
	type = "RRT Carrot Planner";
	return true;
}


bool RRTCarrot::mergePath(node_ptr_t path_1_node, node_ptr_t path_2_node)
{
	ROS_INFO_STREAM("Merging Path With Bridge At ("<<path_1_node->location_.x<<"."<<path_1_node->location_.y<<
			") and ("<<path_2_node->location_.x<<","<<path_2_node->location_.y<<")");
	node_ptr_t new_parent_node(path_1_node);
	node_ptr_t new_child_node(path_2_node);
	node_ptr_t swap;
	if(path_2_node == node_ptr_t() || path_1_node == node_ptr_t())
	{
		return false;
	}
	while(new_child_node!=node_ptr_t())
	{
		ROS_INFO_STREAM("Connecting ("<<new_child_node->location_.x<<"."<<new_child_node->location_.y<<
				") and ("<<new_parent_node->location_.x<<","<<new_parent_node->location_.y<<")");
		swap                    = new_child_node->parent_;
		new_child_node->parent_ = new_parent_node;
		new_parent_node         = new_child_node;
		new_child_node          = swap;

	}
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
