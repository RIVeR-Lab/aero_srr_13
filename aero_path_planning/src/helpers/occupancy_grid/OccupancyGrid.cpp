/**
 * @file	OccupancyGrid.cpp
 * @date	Oct 22, 2012
 * @author	Adam Panzica
 * @brief	Contains the implementations for OccupancyGrid.h
 */

#include <boost/lexical_cast.hpp>
#include<tf/transform_datatypes.h>
#include<pcl/io/io.h>



#include <aero_path_planning/occupancy_grid/OccupancyGrid.h>

namespace gm = geometry_msgs;
using namespace app;


OccupancyGrid::OccupancyGrid(): occ_grid_()
{
	this->x_dim_	= 0;
	this->y_dim_	= 0;
	this->z_dim_	= 0;
	this->res_	= 0;
	this->has_goal_ = false;
}

OccupancyGrid::OccupancyGrid(const OccupancyGrid& grid):
														origin_(grid.origin_),
														occ_grid_(grid.occ_grid_),
														converter_(grid.converter_)
{
	this->x_dim_	= grid.x_dim_;
	this->y_dim_	= grid.y_dim_;
	this->z_dim_	= grid.z_dim_;
	this->res_	    = grid.res_;
	this->has_goal_ = grid.has_goal_;
	this->goal_     = grid.goal_;
};


OccupancyGrid::OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const app::Point& origin, PointTrait_t seedTrait,  const std::string& frame_id):
														origin_(origin),
														occ_grid_((xDim+1)*(yDim+1)*(zDim+1),1),
														converter_(resolution)
{
	this->intializeDim(xDim, yDim, zDim);
	this->res_	= resolution;
	this->has_goal_ = false;
	this->occ_grid_.header.frame_id = frame_id;

	//ROS_INFO("Calculated Occupancy Grid Size: %d", this->occGrid.get()->size());
	//Initialize the grid
	intializeGrid(seedTrait);

}

OccupancyGrid::OccupancyGrid(int xDim, int yDim, double resolution, const app::Point& origin, PointTrait_t seedTrait,  const std::string& frame_id):
														origin_(origin),
														occ_grid_((xDim+1)*(yDim+1),1),
														converter_(resolution)
{
	this->intializeDim(xDim, yDim, 0);
	this->res_	= resolution;
	this->has_goal_ = false;
	this->occ_grid_.header.frame_id = frame_id;

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
OccupancyGrid::OccupancyGrid(int xDim, int yDim, int zDim, double resolution, const app::Point& origin, const OccupancyGridCloud& cloud) throw(OccupancyGridAccessException):
														origin_(origin),
														occ_grid_((xDim+1)*(yDim+1)*(zDim+1),1),
														converter_(resolution)
{
	//ROS_INFO("Generating new Point Cloud Based Occupancy Grid With Parameters: <%d, %d, %d>", xDim, yDim, zDim);
	//ROS_INFO("Received cloud Should be Size <%d>, is size <%d>",(int)this->occ_grid_.size(), (int)cloud.size());
	this->intializeDim(xDim, yDim, zDim);
	this->res_	    = resolution;
	this->has_goal_ = false;
	this->occ_grid_.header.frame_id = cloud.header.frame_id;

#pragma omp parallel for
	for(unsigned int index = 0; index<cloud.size(); index++)
	{
		//ROS_INFO("Extracted Data <%f, %f, %f, %x>", cloud.at(index).x, cloud.at(index).y, cloud.at(index).z, cloud.at(index).rgba);
		setPoint(cloud.at(index), false);
		//ROS_INFO("Set Data <%f, %f, %f, %x>", getPoint(cloud.at(index), false).x, getPoint(cloud.at(index), false).y, getPoint(cloud.at(index), false).z, getPoint(cloud.at(index), false).rgba);
	}


}

OccupancyGrid::OccupancyGrid(const app::OccupancyGridMsg& message):
													occ_grid_()
{
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
	this->occ_grid_.header.frame_id = message.header.frame_id;
}

void OccupancyGrid::intializeDim(int x_dim, int y_dim, int z_dim)
{
	this->x_dim_ = x_dim+1;
	this->y_dim_ = y_dim+1;
	this->z_dim_ = z_dim+1;
}

void OccupancyGrid::intializeGrid(PointTrait_t seedTrait)
{
	//ROS_INFO("Grid Size:%d", (int)this->occ_grid_.size());
#pragma omp parallel for	//pragma that tells OpenMP to parallelize this loop
	for(int x=0; x<this->x_dim_; x++)
	{
		for(int y=0; y<this->y_dim_; y++)
		{
			for(int z=0; z<this->z_dim_; z++)
			{
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

void OccupancyGrid::searchForGoal()
{
#pragma omp parallel for
	for(int i = 0; i< (int)this->occ_grid_.size(); i++)
	{
		if(this->occ_grid_.at(i).rgba==app::GOAL)
		{
			this->goal_     = this->occ_grid_.at(i);
			this->has_goal_ = true;
		}
	}
}

OccupancyGrid::~OccupancyGrid(){}

PointTrait OccupancyGrid::getPointTrait(int x, int y, int z)const throw(OccupancyGridAccessException)
				{
	Point workingPoint;
	workingPoint.x = x;
	workingPoint.y = y;
	workingPoint.z = z;
	return getPointTrait(workingPoint);
				}

PointTrait OccupancyGrid::getPointTrait(const Point& point)const throw(OccupancyGridAccessException)
				{
	Point corrected_point(point);
	corrected_point.getVector4fMap()+=this->origin_.getVector4fMap();
	if(boundsCheck(corrected_point))
	{
		try
		{
			return static_cast<app::PointTrait>(getPoint(corrected_point).rgba);
		}
		catch(std::exception& e)
		{
			std::string message("Internal Point Cloud Problem!");
			OccupancyGridAccessException e1(message , e);
			throw e1;
		}
	}
	return app::UNKNOWN;
				}


bool OccupancyGrid::setPointTrait(int x, int y, int z, PointTrait trait)throw(OccupancyGridAccessException)
{
	Point working_point;
	working_point.x = x;
	working_point.y = y;
	working_point.z = z;
	working_point.rgba = trait;
	return setPointTrait(working_point);
}

bool OccupancyGrid::setPointTrait(const Point& point)throw(OccupancyGridAccessException)
{
	Point corrected_point(point);
	corrected_point.getVector4fMap()+=this->origin_.getVector4fMap();
	if(boundsCheck(corrected_point))
	{
		this->getPoint(corrected_point).rgba = point.rgba;
		if(point.rgba==app::GOAL)
		{
			this->goal_     = this->getPoint(corrected_point);
			this->has_goal_ = true;
		}
		return true;
	}
	return false;
}

bool OccupancyGrid::setPointTrait(const app::PointCloud& points) throw(OccupancyGridAccessException)
{
	bool sucess = true;
//#pragma omp parallel for
	for (int i = 0; i < (int)points.size(); i++)
	{
		try
		{
			this->setPointTrait(points.at(i));
		}
		catch(std::runtime_error& e1)
		{
			sucess = false;
		}
	}
	if(sucess)
	{
		return sucess;
	}
	else
	{
		std::string message("Could not copy a point in the cloud onto the grid as it was out of bounds");
		OccupancyGridAccessException e(message);
		throw e;
	}
}

bool OccupancyGrid::setPointTrait(const nm::OccupancyGrid& points, bool clipping, bool scaling) throw(OccupancyGridAccessException)
				{
	bool success = true;
	bool bound   = false;
	double scale = this->res_/points.info.resolution;

	//Create a point converter to go from occupancy_grid scale to local scale
	app::PointConverter converter(scale);

	//Check for clipping
	if(!clipping)
	{
		bool xbound  = double(points.info.width)*scale  < this->x_dim_;
		bool ybound  = double(points.info.height)*scale < this->y_dim_;
		bound = xbound&&ybound;
	}
	else
	{
		bound = true;
	}

	if(bound)
	{
		Point origin_offset;
		Point origin_patch;
		origin_patch.x = points.info.origin.position.x;
		origin_patch.y = points.info.origin.position.y;
		converter.convertToGrid(origin_patch, origin_patch);
		//Calculate the offset in origins between the grids, if there is one
		origin_offset.getVector4fMap() = this->origin_.getVector4fMap() - origin_patch.getVector4fMap();

#pragma omp parallel for
		for(int x = 0; x<(int)points.info.width; x++)
		{
			for(int y = 0; y<(int)points.info.width; y++)
			{
				int point_idx = this->calcIndex(x, y, 0);
				app::Point patch_point;
				patch_point.x = x;
				patch_point.y = y;
				converter.convertToGrid(patch_point, patch_point);
				patch_point.getVector4fMap() += origin_offset.getVector4fMap();
				patch_point.rgba = app::UNKNOWN;
				if(points.data.at(point_idx) > 80)
				{
					patch_point.rgba = app::OBSTACLE;
				}
				else if(points.data.at(point_idx)>0)
				{
					patch_point.rgba = app::FREE_LOW_COST;
				}
				this->setPointTrait(patch_point);
			}
		}
	}
	else
	{
		success = false;
	}


	return success;
				}


const Point& OccupancyGrid::getGoalPoint() const throw (bool)
				{
	if(this->has_goal_)
	{
		return this->goal_;
	}
	else throw false;
				}

void OccupancyGrid::setGoalPoint(app::Point point) throw(OccupancyGridAccessException)
				{
	point.rgba = app::GOAL;
	try
	{
		setPoint(point, false);
	}
	catch(std::exception& e)
	{
		//Do nothing. It just means the goal point wasn't actually reachable on the local map, but it will still work
	}
	this->goal_ = point;
	this->has_goal_ = true;
				}

const OccupancyGridCloud& OccupancyGrid::getGrid() const
{
	return this->occ_grid_;
}

bool OccupancyGrid::generateMessage(sensor_msgs::PointCloud2Ptr message) const
{
	message->header.frame_id = this->occ_grid_.header.frame_id;
	message->header.stamp    = ros::Time::now();
	pcl::toROSMsg<pcl::PointXYZRGBA>(this->occ_grid_, *message);
	return true;
}

bool OccupancyGrid::generateMessage(app::OccupancyGridMsg& message) const
{
	message.header.frame_id = this->occ_grid_.header.frame_id;
	message.header.stamp    = ros::Time::now();
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

bool OccupancyGrid::generateMessage(sensor_msgs::Image& message) const
{
	ROS_WARN("Images Not Currently Supported!");
	return false;
	/*try
	{
		pcl::toROSMsg(this->occ_grid_, message);
	}
	catch(std::runtime_error& e)
	{
		ROS_ERROR("Problem building an Image from the OccupancyGrid: %s", e.what());
		return false;
	}
	return true;*/
}

OccupancyGrid::iterator OccupancyGrid::begin()
{
	return this->occ_grid_.begin();
}

OccupancyGrid::iterator OccupancyGrid::end()
{
	return this->occ_grid_.end();
}

OccupancyGrid::const_iterator OccupancyGrid::cbegin() const
{
	return this->occ_grid_.begin();
}

OccupancyGrid::const_iterator OccupancyGrid::cend() const
{
	return this->occ_grid_.end();
}

const app::PointConverter& OccupancyGrid::getConverter() const
{
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
boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, int slice) const
{
	boost::shared_ptr<std::string> output(new std::string(""));
	std::vector<std::string> occ_grid(this->x_dim_, std::string(this->y_dim_, ' '));

	char value[2] ={0,'\0'};
	for(int x=0; x<this->x_dim_; x++)
	{
		for(int y=0; y<this->y_dim_; y++)
		{
			switch(getPoint(x, y, 0).rgba)
			{
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
			occ_grid.at(x).replace(y,1,value);
		}
	}

	for(unsigned int i=0;i<occ_grid.size(); i++)
	{
		*output+=occ_grid.at(i);
		*output+="\r\n";
	}
	return output;
}

int OccupancyGrid::calcIndex(int x, int y, int z) const
{
	return x + (this->y_dim_) * (y + (this->z_dim_) * z);
}

const Point& OccupancyGrid::getPoint(const app::Point& point, bool origin_corrected) const
{
	if(origin_corrected)
	{
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}
	else
	{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

const Point& OccupancyGrid::getPoint(app::Point& point, bool origin_corrected) const
{
	if(origin_corrected)
	{
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}
	else
	{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

const Point& OccupancyGrid::getPoint(int x, int y, int z) const
{
	return this->occ_grid_.at(calcIndex(x, y, z));
}

Point& OccupancyGrid::getPoint(app::Point& point, bool origin_corrected)
{
	if(origin_corrected)
	{
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}
	else
	{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

Point& OccupancyGrid::getPoint(const app::Point& point, bool origin_corrected)
{
	if(origin_corrected)
	{
		return this->occ_grid_.at(calcIndex(point.x, point.y, point.z));
	}
	else
	{
		return this->occ_grid_.at(calcIndex(point.x+this->origin_.x, point.y+this->origin_.y, point.z+this->origin_.z));
	}
}

Point& OccupancyGrid::getPoint(int x, int y, int z)
{
	return this->occ_grid_.at(calcIndex(x, y, z));
}


bool OccupancyGrid::boundsCheck(const Point& point)const throw(OccupancyGridAccessException)
								{
	bool failure = false;
	const std::string prefex("Invalid Point Requested: ");
	const std::string middle(" Is Greater Than Max Value Or Less Than Zero: ");
	std::stringstream message("");
	if(point.x>this->x_dim_ || point.x<0)
	{
		message<<prefex<<"X value"<<point.x<<middle<<this->x_dim_;
		failure = true;
	}
	else if(point.y>this->y_dim_|| point.x<0)
	{
		message<<prefex<<"Y value"<<point.y<<middle<<this->y_dim_;
		failure = true;
	}
	else if(point.z>this->z_dim_|| point.x<0)
	{
		message<<prefex<<"Z value"<<point.z<<middle<<this->z_dim_;
		failure = true;
	}
	if(failure)
	{
		std::string message_out(message.str());
		OccupancyGridAccessException exception(message_out);
		throw exception;
	}
	return !failure;
								}

void OccupancyGrid::setPoint(const Point& copy_point, bool origin_corrected)
{
	Point& grid_point = getPoint(copy_point, origin_corrected);
	grid_point.x = copy_point.x;
	grid_point.y = copy_point.y;
	grid_point.z = copy_point.z;
	grid_point.rgba = copy_point.rgba;
	if(grid_point.rgba == app::GOAL)
	{
		this->goal_		= grid_point;
		this->has_goal_	= true;
	}
}

unsigned long OccupancyGrid::size() const
{
	return this->x_dim_*this->y_dim_*this->z_dim_;
}


int OccupancyGrid::getXSize() const
{
	return this->x_dim_-1;
}

int OccupancyGrid::getYSize() const
{
	return this->y_dim_-1;
}

int OccupancyGrid::getZSize() const
{
	return this->z_dim_-1;
}

const Point& OccupancyGrid::getOriginPoint() const
{
	return this->origin_;
}

const std::string& OccupancyGrid::getFrameId() const
{
	return this->occ_grid_.header.frame_id;
}

bool OccupancyGrid::isValidPoint(const Point& point, bool origin_corrected) const
{
	Point corrected_point(point);
	if(!origin_corrected)
	{
		corrected_point.getVector4fMap()+=this->origin_.getVector4fMap();
	}
	try
	{
		return this->boundsCheck(corrected_point);
	}
	catch(std::exception& e)
	{
		return false;
	}
}

