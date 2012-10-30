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

OccupancyGrid::OccupancyGrid(): occGrid(){
	this->xDim	= 0;
	this->yDim	= 0;
	this->zDim	= 0;
	this->res	= 0;
	this->xSize = 0;
	this->ySize = 0;
	this->zSize = 0;
	this->x_ori = 0;
	this->y_ori = 0;
	this->z_ori = 0;
}

OccupancyGrid::OccupancyGrid(OccupancyGrid& grid):
								origin(grid.origin),
								occGrid(grid.occGrid){
	this->xDim	= grid.xDim;
	this->yDim	= grid.yDim;
	this->zDim	= grid.zDim;
	this->res	= grid.res;
	this->xSize = grid.xSize;
	this->ySize = grid.ySize;
	this->zSize = grid.zSize;
	this->x_ori = grid.x_ori;
	this->y_ori = grid.y_ori;
	this->z_ori = grid.z_ori;
};

OccupancyGrid::OccupancyGrid(const OccupancyGrid& grid):
								origin(grid.origin),
								occGrid(grid.occGrid){
	this->xDim	= grid.xDim;
	this->yDim	= grid.yDim;
	this->zDim	= grid.zDim;
	this->res	= grid.res;
	this->xSize = grid.xSize;
	this->ySize = grid.ySize;
	this->zSize = grid.zSize;
	this->x_ori = grid.x_ori;
	this->y_ori = grid.y_ori;
	this->z_ori = grid.z_ori;
};


OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, const oryx_path_planning::Point& origin, PointTrait_t seedTrait):
								origin(origin),
								occGrid((roundToGrid(xDim, resolution)+1)*(roundToGrid(yDim, resolution)+1)*(roundToGrid(zDim, resolution)+1),1){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= roundToFrac(zDim, resolution);
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	this->zSize = roundToGrid(this->zDim, this->res)+1;
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);
	this->z_ori = roundToGrid(this->origin.z, this->res);

	//ROS_INFO("Calculated Occupancy Grid Size: %d", this->occGrid.get()->size());
	//Initialize the grid
	intializeGrid(seedTrait);

}

OccupancyGrid::OccupancyGrid(double xDim, double yDim, double resolution, const oryx_path_planning::Point& origin, PointTrait_t seedTrait):
								origin(origin),
								occGrid((roundToGrid(xDim, resolution)+1)*(roundToGrid(yDim, resolution)+1),1)
{
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= 0;
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	this->zSize = 1;
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);
	this->z_ori = 0;

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
OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, const oryx_path_planning::Point& origin, const OccpancyGridCloud& cloud) throw(OccupancyGridAccessException):
								origin(origin),
								occGrid((roundToGrid(xDim, resolution)+1)*(roundToGrid(yDim, resolution)+1)*(roundToGrid(zDim, resolution)+1),1){
	ROS_INFO("Generating new Point Cloud Based Occupancy Grid With Parameters: <%f, %f, %f>", xDim, yDim, zDim);
	ROS_INFO("Received cloud Should be Size <%d>, is size <%d>",(int)this->occGrid.size(), (int)cloud.size());
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	//Check for a 2D PointCloud
	if(this->zDim==0){
		this->zDim	= 0;
		this->zSize = 1;
		this->z_ori = 0;
	}else{
		this->zDim	= roundToFrac(zDim, resolution);
		this->zSize = roundToGrid(this->zDim, this->res)+1;
		this->z_ori = roundToGrid(this->origin.z, this->res);
	}
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);

	//OccupancyGrid::iterator occItr = this->occGrid->begin();
	/*for(cloudItr = cloud->begin(); cloudItr<cloud->end(); cloudItr++){
		ROS_INFO("Point Cloud Point <%f, %f, %f, %x>", cloudItr->x, cloudItr->y, cloudItr->z, cloudItr->rgba);
		ROS_INFO("Occupancy Grid Point <%f, %f, %f, %x>", occItr->x, occItr->y, occItr->z, occItr->rgba);
		occItr++;
	}*/

	for(unsigned int index = 0; index<cloud.size(); index++){
		//ROS_INFO("Extracted Data <%f, %f, %f, %x>", cloud.at(index).x, cloud.at(index).y, cloud.at(index).z, cloud.at(index).rgba);
		setPoint(cloud.at(index), false);
		//ROS_INFO("Set Data <%f, %f, %f, %x>", getPoint(cloud.at(index), false).x, getPoint(cloud.at(index), false).y, getPoint(cloud.at(index), false).z, getPoint(cloud.at(index), false).rgba);
	}

	/*occItr = this->occGrid->begin();
	for(cloudItr = cloud->begin(); cloudItr<cloud->end(); cloudItr++){
		ROS_INFO("Point Cloud Point <%f, %f, %f, %x>", cloudItr->x, cloudItr->y, cloudItr->z, cloudItr->rgba);
		ROS_INFO("Occupancy Grid Point <%f, %f, %f, %x>", occItr->x, occItr->y, occItr->z, occItr->rgba);
		occItr++;
	}*/
	//ROS_INFO("Built the Occupancy Grid <%f/%d, %f/%d, %f/%d>:\n%s",this->xDim, this->xSize,this->yDim, this->ySize,this->zDim, this->zSize, this->toString(0,0)->c_str());

}

/**
 * This constructor takes a PointCloud, which may or may not have its points in any particular order, and copies it into
 * the PointCloud which backs this occupancy grid. In the process it organizes the points in such a fashion as to make them
 * accessible quickly by <x,y,z> coordinates without the need for iteration and comparison. Since there is no built in provision
 * to bounds-check the XYZ data in the Points in the given PointCloud, this constructor will throw an exception if a Point in
 * the PointCloud falls outside the grid specified by the xDim, yDim and zDim parameters.
 */
OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, oryx_path_planning::Point& origin, OccpancyGridCloud& cloud) throw(OccupancyGridAccessException):
								origin(origin),
								occGrid((roundToGrid(xDim, resolution)+1)*(roundToGrid(yDim, resolution)+1)*(roundToGrid(zDim, resolution)+1),1){
	ROS_INFO("Generating new Point Cloud Based Occupancy Grid With Parameters: <%f, %f, %f>", xDim, yDim, zDim);
	ROS_INFO("Received cloud Should be Size <%d>, is size <%d>",(int)this->occGrid.size(), (int)cloud.size());
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	//Check for a 2D PointCloud
	if(this->zDim==0){
		this->zDim	= 0;
		this->zSize = 1;
		this->z_ori = 0;
	}else{
		this->zDim	= roundToFrac(zDim, resolution);
		this->zSize = roundToGrid(this->zDim, this->res)+1;
		this->z_ori = roundToGrid(this->origin.z, this->res);
	}
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);

	//OccupancyGrid::iterator occItr = this->occGrid->begin();
	/*for(cloudItr = cloud->begin(); cloudItr<cloud->end(); cloudItr++){
		ROS_INFO("Point Cloud Point <%f, %f, %f, %x>", cloudItr->x, cloudItr->y, cloudItr->z, cloudItr->rgba);
		ROS_INFO("Occupancy Grid Point <%f, %f, %f, %x>", occItr->x, occItr->y, occItr->z, occItr->rgba);
		occItr++;
	}*/


	for(unsigned int index = 0; index<cloud.size(); index++){
		//ROS_INFO("Extracted Data <%f, %f, %f, %x>", cloud.at(index).x, cloud.at(index).y, cloud.at(index).z, cloud.at(index).rgba);
		setPoint(cloud.at(index), false);
		//ROS_INFO("Set Data <%f, %f, %f, %x>", getPoint(cloud.at(index), false).x, getPoint(cloud.at(index), false).y, getPoint(cloud.at(index), false).z, getPoint(cloud.at(index), false).rgba);
	}

	/*occItr = this->occGrid->begin();
	for(cloudItr = cloud->begin(); cloudItr<cloud->end(); cloudItr++){
		ROS_INFO("Point Cloud Point <%f, %f, %f, %x>", cloudItr->x, cloudItr->y, cloudItr->z, cloudItr->rgba);
		ROS_INFO("Occupancy Grid Point <%f, %f, %f, %x>", occItr->x, occItr->y, occItr->z, occItr->rgba);
		occItr++;
	}*/
	//ROS_INFO("Built the Occupancy Grid <%f/%d, %f/%d, %f/%d>:\n%s",this->xDim, this->xSize,this->yDim, this->ySize,this->zDim, this->zSize, this->toString(0,0)->c_str());

}

OccupancyGrid::OccupancyGrid(oryxsrr_msgs::OccupancyGridPtr& message):
							occGrid(){
	this->xDim	= roundToFrac(message->xDim, message->res);
	this->yDim	= roundToFrac(message->yDim, message->res);
	this->zDim	= roundToFrac(message->zDim, message->res);
	this->res	= message->res;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);
	//Check for a 2D PointCloud
	if(this->zDim!=0){
		this->zSize = roundToGrid(this->zDim, this->res)+1;
		this->z_ori = roundToGrid(this->origin.z, this->res);
	}
	else{
		this->zSize = 1;
		this->z_ori = 0;
	}
	pcl::fromROSMsg<pcl::PointXYZRGBA>(message->cloud, this->occGrid);
}

OccupancyGrid::OccupancyGrid(oryxsrr_msgs::OccupancyGridConstPtr& message):
							occGrid(){
	this->xDim	= roundToFrac(message->xDim, message->res);
	this->yDim	= roundToFrac(message->yDim, message->res);
	this->zDim	= roundToFrac(message->zDim, message->res);
	this->res	= message->res;
	this->xSize = roundToGrid(this->xDim, this->res)+1;
	this->ySize = roundToGrid(this->yDim, this->res)+1;
	this->x_ori = roundToGrid(this->origin.x, this->res);
	this->y_ori = roundToGrid(this->origin.y, this->res);
	//Check for a 2D PointCloud
	if(this->zDim!=0){
		this->zSize = roundToGrid(this->zDim, this->res)+1;
		this->z_ori = roundToGrid(this->origin.z, this->res);
	}
	else{
		this->zSize = 1;
		this->z_ori = 0;
	}
	pcl::fromROSMsg<pcl::PointXYZRGBA>(message->cloud, this->occGrid);
}

void OccupancyGrid::intializeGrid(PointTrait_t seedTrait){
	for(int x=0; x<this->xSize; x++){
		for(int y=0; y<this->ySize; y++){
			for(int z=0; z<this->zSize; z++){
				//ROS_INFO("Initializing Point <%d, %d, %d>", x,y,z);
				//Initialize the point
				Point& point = getPoint(x,y,z);
				point.x = gridToReal(x-this->origin.x, this->res);
				point.y = gridToReal(y-this->origin.y, this->res);
				point.z = gridToReal(z-this->origin.z, this->res);
				point.rgba = seedTrait;
			}
		}
	}
}

OccupancyGrid::~OccupancyGrid(){}

PointTrait OccupancyGrid::getPointTrait(double x, double y, double z)const throw(OccupancyGridAccessException){
	Point workingPoint;
	workingPoint.x = x;
	workingPoint.y = y;
	workingPoint.z = z;
	return getPointTrait(workingPoint);
}

PointTrait OccupancyGrid::getPointTrait(Point point)const throw(OccupancyGridAccessException){
	point.x=roundToFrac(point.x+this->origin.x, this->res);
	point.y=roundToFrac(point.y+this->origin.y, this->res);
	point.z=roundToFrac(point.z+this->origin.z, this->res);
	if(boundsCheck(point)){
		try{
			return static_cast<oryx_path_planning::PointTrait>(getPoint(point).rgba);
		}catch(std::exception& e){
			throw *(new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e));
		}
	}
	return oryx_path_planning::UNKNOWN;
}


bool OccupancyGrid::setPointTrait(double x, double y, double z, PointTrait trait)throw(OccupancyGridAccessException){
	Point workingPoint;
	workingPoint.x = x;
	workingPoint.y = y;
	workingPoint.z = z;
	return setPointTrait(workingPoint, trait);
}

bool OccupancyGrid::setPointTrait(Point point, PointTrait trait)throw(OccupancyGridAccessException){
	point.x=roundToFrac(point.x+this->origin.x, this->res);
	point.y=roundToFrac(point.y+this->origin.y, this->res);
	point.z=roundToFrac(point.z+this->origin.z, this->res);
	if(boundsCheck(point)){
		try{
			getPoint(point).rgba=trait;
			return true;
		}catch(std::exception& e){
			throw *(new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e));
		}
	}
	return false;
}

const OccpancyGridCloud& OccupancyGrid::getGrid() const{
	return this->occGrid;
}

bool OccupancyGrid::generateMessage(sensor_msgs::PointCloud2Ptr message) const{
	message->header.stamp = ros::Time::now();
	pcl::toROSMsg<pcl::PointXYZRGBA>(this->occGrid, *message);
	return true;
}

bool OccupancyGrid::generateMessage(oryxsrr_msgs::OccupancyGridPtr message) const{
	message->header.stamp = ros::Time::now();
	message->xDim = this->xDim;
	message->yDim = this->yDim;
	message->zDim = this->zDim;
	message->res  = this->res;
	pcl::toROSMsg<pcl::PointXYZRGBA>(this->occGrid, message->cloud);
	return true;
}

OccupancyGrid::iterator OccupancyGrid::begin(){
	return this->occGrid.begin();
}

OccupancyGrid::iterator OccupancyGrid::end(){
	return this->occGrid.end();
}

OccupancyGrid::const_iterator OccupancyGrid::begin() const{
	return this->occGrid.begin();
}

OccupancyGrid::const_iterator OccupancyGrid::end() const{
	return this->occGrid.end();
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
boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, double slice) const{
	boost::shared_ptr<std::string> output(new std::string(""));
	std::vector<std::string> occGrid(this->xSize, std::string(this->ySize, ' '));

	char value[2] ={0,'\0'};
	for(int x=0; x<this->xSize; x++){
		for(int y=0; y<this->ySize; y++){
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

int OccupancyGrid::calcIndex(double x, double y, double z) const{
	int xVal = roundToGrid(x, this->res);
	int yVal = roundToGrid(y, this->res);
	int zVal = roundToGrid(z, this->res);

	return calcIndex(xVal, yVal, zVal);
}

int OccupancyGrid::calcIndex(int x, int y, int z) const{
	return (x)*this->zSize*this->ySize + (y)*this->zSize + (z);
}

const Point& OccupancyGrid::getPoint(const oryx_path_planning::Point& point, bool origin_corrected) const{
	if(origin_corrected){
		return this->occGrid.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occGrid.at(calcIndex(point.x+this->origin.x, point.y+this->origin.y, point.z+this->origin.z));
	}
}

const Point& OccupancyGrid::getPoint(oryx_path_planning::Point& point, bool origin_corrected) const{
	if(origin_corrected){
		return this->occGrid.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occGrid.at(calcIndex(point.x+this->origin.x, point.y+this->origin.y, point.z+this->origin.z));
	}
}

const Point& OccupancyGrid::getPoint(int x, int y, int z) const{
	return this->occGrid.at(calcIndex(x, y, z));
}

Point& OccupancyGrid::getPoint(oryx_path_planning::Point& point, bool origin_corrected){
	if(origin_corrected){
		return this->occGrid.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occGrid.at(calcIndex(point.x+this->origin.x, point.y+this->origin.y, point.z+this->origin.z));
	}
}

Point& OccupancyGrid::getPoint(const oryx_path_planning::Point& point, bool origin_corrected){
	if(origin_corrected){
		return this->occGrid.at(calcIndex(point.x, point.y, point.z));
	}else{
		return this->occGrid.at(calcIndex(point.x+this->origin.x, point.y+this->origin.y, point.z+this->origin.z));
	}
}

Point& OccupancyGrid::getPoint(int x, int y, int z){
	return this->occGrid.at(calcIndex(x, y, z));
}


bool OccupancyGrid::boundsCheck(Point& point)const throw(OccupancyGridAccessException){
	bool failure = false;
	const std::string prefex("Invalid Point Requested: ");
	const std::string middle(" Is Greater Than Max Value Or Less Than Zero: ");
	std::stringstream message("");
	if(point.x>this->xDim || point.x<0){
		message<<prefex<<"X value"<<point.x<<middle<<this->xDim;
		failure = true;
	}
	else if(point.y>this->yDim|| point.x<0){
		message<<prefex<<"Y value"<<point.y<<middle<<this->yDim;
		failure = true;
	}
	else if(point.z>this->zDim|| point.x<0){
		message<<prefex<<"Z value"<<point.z<<middle<<this->zDim;
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
}

void OccupancyGrid::setPoint(const Point& copy_point, bool origin_corrected){
	Point& gridPoint = getPoint(copy_point, origin_corrected);
	gridPoint.x = copy_point.x;
	gridPoint.y = copy_point.y;
	gridPoint.z = copy_point.z;
	gridPoint.rgba = copy_point.rgba;
}

};


