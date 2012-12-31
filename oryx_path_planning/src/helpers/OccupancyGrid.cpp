/**
 * @file	OccupancyGrid.cpp
 * @date	Oct 22, 2012
 * @author	Adam Panzica
 * @brief	Contains the implementations for OccupancyGrid.h
 */

#include <boost/lexical_cast.hpp>
#include<tf/transform_datatypes.h>
#include<pcl/io/io.h>



#include "OccupancyGrid.h"

using namespace oryx_path_planning;

namespace oryx_path_planning{

OccupancyGrid::OccupancyGrid(): occ_grid_(){
	this->x_dim_	= 0;
	this->y_dim_	= 0;
	this->z_dim_	= 0;
	this->res_	= 0;
	this->has_goal_ = false;
}

OccupancyGrid::OccupancyGrid(OccupancyGrid& grid):
								origin_(grid.origin_),
								occ_grid_(grid.occ_grid_),
								converter_(grid.converter_){
	this->x_dim_	= grid.x_dim_;
	this->y_dim_	= grid.y_dim_;
	this->z_dim_	= grid.z_dim_;
	this->res_	= grid.res_;
	this->has_goal_ = false;
};

OccupancyGrid::OccupancyGrid(const OccupancyGrid& grid):
								origin_(grid.origin_),
								occ_grid_(grid.occ_grid_),
								converter_(grid.converter_){
	this->x_dim_	= grid.x_dim_;
	this->y_dim_	= grid.y_dim_;
	this->z_dim_	= grid.z_dim_;
	this->res_	= grid.res_;
	this->has_goal_ = false;
};


OccupancyGrid::OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const oryx_path_planning::Point& origin, PointTrait_t seedTrait):
								origin_(origin),
								occ_grid_((xDim+1)*(yDim+1)*(zDim+1),1),
								converter_(resolution){
	this->x_dim_	= xDim;
	this->y_dim_	= yDim;
	this->z_dim_	= zDim;
	this->res_	= resolution;
	this->has_goal_ = false;

	//ROS_INFO("Calculated Occupancy Grid Size: %d", this->occGrid.get()->size());
	//Initialize the grid
	intializeGrid(seedTrait);

}

OccupancyGrid::OccupancyGrid(int xDim, int yDim, double resolution, const oryx_path_planning::Point& origin, PointTrait_t seedTrait):
								origin_(origin),
								occ_grid_((xDim+1)*(yDim+1),1),
								converter_(resolution){
	this->x_dim_	= xDim;
	this->y_dim_	= xDim;
	this->z_dim_	= 0;
	this->res_	= resolution;
	this->has_goal_ = false;

	//ROS_INFO("Calculated Occupancy Grid Size: %d", this->occGrid.get()->size());
	//Initialize the grid
	intializeGrid(seedTrait);
}

/**
 * This constructor takes a PointCloud, which may or may not have its points in any particular order, and copies it into
 * the PointCloud which backs this occupancy grid. In the process it organizes the points in such a fashion as to make them
 * accessible quickly by <x,y,z> coordinates without the need for iteration and comparison. Since there is no built in provision
 * to bounds-check the XYZ data in the Points in the given PointCloud, this constructor will throw an exception if a Point in
 * the PointCloud falls outside the grid specified by the xDim, yDim and zDim parameters.
 */
OccupancyGrid::OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const oryx_path_planning::Point& origin, const OccupancyGridCloud& cloud) throw(OccupancyGridAccessException):
								origin_(origin),
								occ_grid_((xDim+1)*(yDim+1)*(zDim+1),1),
								converter_(resolution){
	ROS_INFO("Generating new Point Cloud Based Occupancy Grid With Parameters: <%d, %d, %d>", xDim, yDim, zDim);
	ROS_INFO("Received cloud Should be Size <%d>, is size <%d>",(int)this->occ_grid_.size(), (int)cloud.size());
	this->x_dim_	= xDim;
	this->y_dim_	= yDim;
	this->z_dim_	= zDim;
	this->res_	= resolution;
	this->has_goal_ = false;

	for(unsigned int index = 0; index<cloud.size(); index++){
		//ROS_INFO("Extracted Data <%f, %f, %f, %x>", cloud.at(index).x, cloud.at(index).y, cloud.at(index).z, cloud.at(index).rgba);
		setPoint(cloud.at(index), false);
		//ROS_INFO("Set Data <%f, %f, %f, %x>", getPoint(cloud.at(index), false).x, getPoint(cloud.at(index), false).y, getPoint(cloud.at(index), false).z, getPoint(cloud.at(index), false).rgba);
	}


}

/**
 * This constructor takes a PointCloud, which may or may not have its points in any particular order, and copies it into
 * the PointCloud which backs this occupancy grid. In the process it organizes the points in such a fashion as to make them
 * accessible quickly by <x,y,z> coordinates without the need for iteration and comparison. Since there is no built in provision
 * to bounds-check the XYZ data in the Points in the given PointCloud, this constructor will throw an exception if a Point in
 * the PointCloud falls outside the grid specified by the xDim, yDim and zDim parameters.
 */
OccupancyGrid::OccupancyGrid(int xDim, int yDim, int zDim, double resolution, oryx_path_planning::Point& origin, OccupancyGridCloud& cloud) throw(OccupancyGridAccessException):
								origin_(origin),
								occ_grid_((xDim+1)*(yDim+1)*(zDim+1),1),
								converter_(resolution){
	ROS_INFO("Generating new Point Cloud Based Occupancy Grid With Parameters: <%d, %d, %d>", xDim, yDim, zDim);
	ROS_INFO("Received cloud Should be Size <%d>, is size <%d>",(int)this->occ_grid_.size(), (int)cloud.size());
	this->x_dim_	= xDim;
	this->y_dim_	= yDim;
	this->z_dim_	= zDim;
	this->res_	= resolution;
	this->has_goal_ = false;

	for(unsigned int index = 0; index<cloud.size(); index++){
		//ROS_INFO("Extracted Data <%f, %f, %f, %x>", cloud.at(index).x, cloud.at(index).y, cloud.at(index).z, cloud.at(index).rgba);
		setPoint(cloud.at(index), false);
		//ROS_INFO("Set Data <%f, %f, %f, %x>", getPoint(cloud.at(index), false).x, getPoint(cloud.at(index), false).y, getPoint(cloud.at(index), false).z, getPoint(cloud.at(index), false).rgba);
	}

}

OccupancyGrid::OccupancyGrid(oryx_path_planning::OccupancyGridMsg& message):
							occ_grid_(){
	//extract grid information
	this->x_dim_	= message.x_dim;
	this->y_dim_	= message.y_dim;
	this->z_dim_	= message.z_dim;
	this->res_		= message.resolution;

	//extract origin data
	this->origin_.x = message.x_origin;
	this->origin_.y = message.y_origin;
	this->origin_.z = message.z_origin;

	//extract goal data
	if(message.has_goal)
	{
		this->has_goal_ = true;
		this->goal_.x   = message.x_goal;
		this->goal_.y   = message.y_goal;
		this->goal_.z   = message.z_goal;
	}
	else this->has_goal_ = false;

	//extract grid data
	pcl::fromROSMsg<pcl::PointXYZRGBA>(message.grid_data, this->occ_grid_);
}

void OccupancyGrid::intializeGrid(PointTrait_t seedTrait){
	ROS_INFO("Grid Size:%d", (int)this->occ_grid_.size());
	for(int x=0; x<this->x_dim_+1; x++){
		for(int y=0; y<this->y_dim_+1; y++){
			for(int z=0; z<this->z_dim_+1; z++){
				//ROS_INFO("Initializing Point <%d, %d, %d>", x,y,z);
				//Initialize the point
				//ROS_INFO("I'm Trying To Initialize Point %d,%d,%d, Index=%d", x,y,z, calcIndex(x,y,z));
				Point& point = getPoint(x,y,z);
				point.x = x-this->origin_.x;
				point.y = y-this->origin_.y;
				point.z = z-this->origin_.z;
				point.rgba = seedTrait;
			}
		}
	}
}

void OccupancyGrid::searchForGoal(){
	for(OccupancyGridCloud::iterator search_itr= this->occ_grid_.begin(); search_itr< this->occ_grid_.end(); search_itr++){
		if(search_itr->rgba==oryx_path_planning::GOAL){
			this->goal_ = *search_itr;
			this->has_goal_ = true;
		}
	}
}

OccupancyGrid::~OccupancyGrid(){}

PointTrait OccupancyGrid::getPointTrait(int x, int y, int z)const throw(OccupancyGridAccessException){
	Point workingPoint;
	workingPoint.x = x;
	workingPoint.y = y;
	workingPoint.z = z;
	return getPointTrait(workingPoint);
}

PointTrait OccupancyGrid::getPointTrait(Point point)const throw(OccupancyGridAccessException){
	point.getVector4fMap()+=this->origin_.getVector4fMap();
	if(boundsCheck(point)){
		try{
			return static_cast<oryx_path_planning::PointTrait>(getPoint(point).rgba);
		}catch(std::exception& e){
			throw *(new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e));
		}
	}
	return oryx_path_planning::UNKNOWN;
}


bool OccupancyGrid::setPointTrait(int x, int y, int z, PointTrait trait)throw(OccupancyGridAccessException){
	Point workingPoint;
	workingPoint.x = x;
	workingPoint.y = y;
	workingPoint.z = z;
	return setPointTrait(workingPoint, trait);
}

bool OccupancyGrid::setPointTrait(Point point, PointTrait trait)throw(OccupancyGridAccessException){
	point.getVector4fMap()+=this->origin_.getVector4fMap();
	if(boundsCheck(point)){
		try{
			oryx_path_planning::Point& setPoint = getPoint(point);
			setPoint.rgba = trait;
			if(trait==oryx_path_planning::GOAL){
				this->goal_ = setPoint;
				this->has_goal_ = true;
			}
			return true;
		}catch(std::exception& e){
			throw *(new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e));
		}
	}
	return false;
}

const Point& OccupancyGrid::getGoalPoint() const throw (bool){
	if(this->has_goal_){
		return this->goal_;
	}else throw false;
}

void OccupancyGrid::setGoalPoint(oryx_path_planning::Point point) throw(OccupancyGridAccessException){
	try{
		setPoint(point, oryx_path_planning::GOAL);
		this->goal_ = point;
		this->has_goal_ = true;
	}catch(std::exception& e){
		std::string message("Unable to Place Goal Point On Grid");
		OccupancyGridAccessException error(message, e);
		throw error;
	}
}

const OccupancyGridCloud& OccupancyGrid::getGrid() const{
	return this->occ_grid_;
}

bool OccupancyGrid::generateMessage(sensor_msgs::PointCloud2Ptr message) const{
	message->header.stamp = ros::Time::now();
	pcl::toROSMsg<pcl::PointXYZRGBA>(this->occ_grid_, *message);
	return true;
}

bool OccupancyGrid::generateMessage(oryx_path_planning::OccupancyGridMsg& message) const{
	message.header.stamp = ros::Time::now();
	//Pack grid data
	message.x_dim = this->x_dim_;
	message.y_dim = this->y_dim_;
	message.z_dim = this->z_dim_;
	message.resolution  = this->res_;
	//Pack origin data
	message.x_origin = this->origin_.x;
	message.y_origin = this->origin_.y;
	message.z_origin = this->origin_.z;
	//pack goal data
	if(this->has_goal_)
	{
		message.has_goal = true;
		message.x_goal   = this->goal_.x;
		message.y_goal   = this->goal_.y;
		message.z_goal   = this->goal_.z;
	}
	else message.has_goal = false;
	//pack grid data
	pcl::toROSMsg<pcl::PointXYZRGBA>(this->occ_grid_, message.grid_data);
	return true;
}

bool OccupancyGrid::generateMessage(sensor_msgs::Image& message) const{
	try{
		pcl::toROSMsg(this->occ_grid_, message);
	}catch(std::runtime_error& e){
		ROS_ERROR("Problem building an Image from the OccupancyGrid: %s", e.what());
		return false;
	}
	return true;
}

OccupancyGrid::iterator OccupancyGrid::begin(){
	return this->occ_grid_.begin();
}

OccupancyGrid::iterator OccupancyGrid::end(){
	return this->occ_grid_.end();
}

OccupancyGrid::const_iterator OccupancyGrid::cbegin() const{
	return this->occ_grid_.begin();
}

OccupancyGrid::const_iterator OccupancyGrid::cend() const{
	return this->occ_grid_.end();
}

const oryx_path_planning::PointConverter& OccupancyGrid::getConverter() const{
	return this->converter_;
}

/**
 * Prints out the specified slice of the occupancy grid in ASCII art with each character representing a point on the grid.
 * For example, with xDim=Ydim=2.5 and resolution = .25, and all points set to UNKNOWN, the returned string will look like
 * this:
 *
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 * UUUUUUUUUU <br>
 *
 * The RGBA to ASCII mapping is as follows:
 * |   RGBA   |  ASCII  |
 * |  :----:  | :-----: |
 * | UNKNOWN  |   'U'   |
 * | FREE_LOW |   ' '   |
 * | FREE_HIGH|   '_'   |
 * | OBSTACLE |   'X'   |
 * | GOAL     |   'G'   |
 * | INFLATED |   'x'   |
 * | Non-Std  |   '*'   |
 */
boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, int slice) const{
	boost::shared_ptr<std::string> output(new std::string(""));
	std::vector<std::string> occGrid(this->x_dim_, std::string(this->y_dim_, ' '));

	char value[2] ={0,'\0'};
	for(int x=0; x<this->x_dim_; x++){
		for(int y=0; y<this->y_dim_; y++){
			switch(getPoint(x, y, 0).rgba){
			case UNKNOWN:
				value[0] = 'U';
				break;
			case FREE_LOW_COST:
				value[0] = ' ';
				break;
			case FREE_HIGH_COST:
				value[0] = '_';
				break;
			case OBSTACLE:
				value[0] = 'X';
				break;
			case GOAL:
				value[0] = 'G';
				break;
			case INFLATED:
				value[0] = 'x';
				break;
			case TENTACLE:
				value[0] = 'T';
				break;
			case ROBOT:
				value[0] = 'R';
				break;
			default:
				value[0] = '*';
				break;
			}
			occGrid.at(x).replace(y,1,value);
		}
	}

	for(unsigned int i=0;i<occGrid.size(); i++){
		*output+=occGrid.at(i);
		*output+="\r\n";
	}
	return output;
}

int OccupancyGrid::calcIndex(int x, int y, int z) const{
	return x + (this->y_dim_+1) * (y + (this->z_dim_+1) * z);
}

const Point& OccupancyGrid::getPoint(const oryx_path_planning::Point& point, bool origin_corrected) const{
	if(origin_corrected){
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

const Point& OccupancyGrid::getPoint(oryx_path_planning::Point& point, bool origin_corrected) const{
	if(origin_corrected){
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

const Point& OccupancyGrid::getPoint(int x, int y, int z) const{
	return this->occ_grid_.at(calcIndex(x, y, z));
}

Point& OccupancyGrid::getPoint(oryx_path_planning::Point& point, bool origin_corrected){
	if(origin_corrected){
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

Point& OccupancyGrid::getPoint(const oryx_path_planning::Point& point, bool origin_corrected){
	if(origin_corrected){
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

Point& OccupancyGrid::getPoint(int x, int y, int z){
	return this->occ_grid_.at(calcIndex(x, y, z));
}


bool OccupancyGrid::boundsCheck(Point& point)const throw(OccupancyGridAccessException){
	bool failure = false;
	const std::string prefex("Invalid Point Requested: ");
	const std::string middle(" Is Greater Than Max Value Or Less Than Zero: ");
	std::stringstream message("");
	if(point.x>this->x_dim_ || point.x<0){
		message<<prefex<<"X value"<<point.x<<middle<<this->x_dim_;
		failure = true;
	}
	else if(point.y>this->y_dim_|| point.x<0){
		message<<prefex<<"Y value"<<point.y<<middle<<this->y_dim_;
		failure = true;
	}
	else if(point.z>this->z_dim_|| point.x<0){
		message<<prefex<<"Z value"<<point.z<<middle<<this->z_dim_;
		failure = true;
	}
	if(failure){
		std::string messageOut(message.str());
		OccupancyGridAccessException exception(messageOut);
		throw exception;
	}
	return true;
}

void OccupancyGrid::setPoint(Point& copy_point, bool origin_corrected){
	Point& gridPoint = getPoint(copy_point, origin_corrected);
	gridPoint.x = copy_point.x;
	gridPoint.y = copy_point.y;
	gridPoint.z = copy_point.z;
	gridPoint.rgba = copy_point.rgba;
	if(gridPoint.rgba == oryx_path_planning::GOAL){
		this->goal_		= gridPoint;
		this->has_goal_	= true;
	}
}

void OccupancyGrid::setPoint(const Point& copy_point, bool origin_corrected){
	Point& gridPoint = getPoint(copy_point, origin_corrected);
	gridPoint.x = copy_point.x;
	gridPoint.y = copy_point.y;
	gridPoint.z = copy_point.z;
	gridPoint.rgba = copy_point.rgba;
	if(gridPoint.rgba == oryx_path_planning::GOAL){
		this->goal_		= gridPoint;
		this->has_goal_	= true;
	}
}

};


