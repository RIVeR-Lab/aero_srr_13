/**
 * @file	OccupancyGrid.cpp
 * @date	Oct 22, 2012
 * @author	Adam Panzica
 * @brief	Contains the implementations for OccupancyGrid.h
 */

#include <boost/lexical_cast.hpp>



#include "OccupancyGrid.h"

using namespace oryx_path_planning;

namespace oryx_path_planning{

OccupancyGrid::OccupancyGrid(): occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>()){
	this->xDim	= 0;
	this->yDim	= 0;
	this->zDim	= 0;
	this->res	= 0;
	this->xSize = roundToGrid(this->xDim, this->res);
	this->ySize = roundToGrid(this->yDim, this->res);
	this->zSize = roundToGrid(this->zDim, this->res);
}

OccupancyGrid::OccupancyGrid(oryx_path_planning::OccupancyGrid& grid){
	this->xDim	= grid.xDim;
	this->yDim	= grid.yDim;
	this->zDim	= grid.zDim;
	this->res	= grid.res;
	this->xSize = grid.xSize;
	this->ySize = grid.ySize;
	this->zSize = grid.zSize;
	this->occGrid = grid.getGrid();
};


OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, PointTrait_t seedTrait): occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>(roundToGrid(xDim, resolution)*roundToGrid(yDim, resolution)*roundToGrid(zDim, resolution),1)){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= roundToFrac(zDim, resolution);
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res);
	this->ySize = roundToGrid(this->yDim, this->res);
	this->zSize = roundToGrid(this->zDim, this->res);

	//ROS_INFO("Calculated Occupancy Grid Size: %d", this->occGrid.get()->size());
	//Initialize the grid
	intializeGrid(seedTrait);

}

OccupancyGrid::OccupancyGrid(double xDim, double yDim, double resolution, PointTrait_t seedTrait): occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>(roundToGrid(xDim, resolution)*roundToGrid(yDim, resolution),1)){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= 0;
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res);
	this->ySize = roundToGrid(this->yDim, this->res);
	this->zSize = 1;

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
OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, pcl::PointCloud<pcl::PointXYZRGBA>& cloud) throw(OccupancyGridAccessException):
		occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>(roundToGrid(xDim, resolution)*roundToGrid(yDim, resolution)*roundToGrid(zDim, resolution),1)){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= roundToFrac(zDim, resolution);
	this->res	= resolution;
	this->xSize = roundToGrid(this->xDim, this->res);
	this->ySize = roundToGrid(this->yDim, this->res);
	//Check for a 2D PointCloud
	if(this->zDim!=0)this->zSize = roundToGrid(this->zDim, this->res);
	else this->zSize = 1;

	//Iterate across the supplied point cloud and fill in the occupancy grid
	for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator iterator(cloud.begin()); iterator<cloud.end(); iterator++){
		pcl::PointXYZRGBA& seedPoint = *iterator;
		pcl::PointXYZRGBA& gridPoint = getPoint(seedPoint.x, seedPoint.y, seedPoint.z);
		gridPoint.x = seedPoint.x;
		gridPoint.y = seedPoint.y;
		gridPoint.z = seedPoint.z;
		gridPoint.rgba = seedPoint.rgba;
	}

}

void OccupancyGrid::intializeGrid(PointTrait_t seedTrait){
	for(int x=0; x<this->xSize; x++){
		for(int y=0; y<this->ySize; y++){
			for(int z=0; z<this->zSize; z++){
				//ROS_INFO("Initializing Point <%d, %d, %d>", x,y,z);
				//Initialize the point
				pcl::PointXYZRGBA& point = getPoint(x,y,z);
				point.x = gridToReal(x, this->res);
				point.y = gridToReal(y, this->res);
				point.z = gridToReal(z, this->res);
				point.rgba = seedTrait;
			}
		}
	}
}

OccupancyGrid::~OccupancyGrid(){}
d
PointTrait OccupancyGrid::getPointTrait(double x, double y, double z)throw(OccupancyGridAccessException){
	if(boundsCheck(x,y,z)){
		try{
			return static_cast<oryx_path_planning::PointTrait>(getPoint(x,y,z).rgba);
		}catch(std::exception& e){
			throw new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e);
		}
	}
	return oryx_path_planning::UNKNOWN;
}


bool OccupancyGrid::setPointTrait(double x, double y, double z, oryx_path_planning::PointTrait_t trait)throw(OccupancyGridAccessException){
	if(boundsCheck(x,y,z)){
		try{
			getPoint(x,y,z).rgba=trait;
			return true;
		}catch(std::exception& e){
			throw new OccupancyGridAccessException(*(new std::string("Internal Point Cloud Problem!")), e);
		}
	}
	return false;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > OccupancyGrid::getGrid(){
	return this->occGrid;
}

bool OccupancyGrid::generateMessage(sensor_msgs::PointCloud2& message){
	pcl::toROSMsg(*this->occGrid, message);
	return true;
}


boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, double slice){
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

int OccupancyGrid::calcIndex(double x, double y, double z){
	int xVal = roundToGrid(x, this->res);
	int yVal = roundToGrid(y, this->res);
	int zVal = roundToGrid(z, this->res);

	return calcIndex(xVal, yVal, zVal);
}

int OccupancyGrid::calcIndex(int x, int y, int z){
	return x*this->zSize*this->ySize + y*this->zSize + z;
}

pcl::PointXYZRGBA& OccupancyGrid::getPoint(double x, double y, double z){
	return this->occGrid.get()->at(calcIndex(x, y, z));
}

pcl::PointXYZRGBA& OccupancyGrid::getPoint(int x, int y, int z){
	return this->occGrid.get()->at(calcIndex(x, y, z));
}

bool OccupancyGrid::boundsCheck(double x, double y, double z)throw(OccupancyGridAccessException){
	bool failure = false;
	const std::string prefex("Invalid Point Requested: ");
	const std::string middle(" Is Greater Than Max Value: ");
	std::string message("");
	if(x>this->xDim){
		boost::lexical_cast<double>(x);
		message+=prefex;
		message+=boost::lexical_cast<double>(x);
		message+=middle;
		message+=boost::lexical_cast<double>(this->xDim);
		failure = true;
	}
	else if(y>this->yDim){
		message+=prefex;
		message+=boost::lexical_cast<double>(y);
		message+=middle;
		message+=boost::lexical_cast<double>(this->yDim);
		failure = true;
	}
	else if(z>this->zDim){
		message+=prefex;
		message+=boost::lexical_cast<double>(z);
		message+=middle;
		message+=boost::lexical_cast<double>(this->zDim);
		failure = true;
	}
	if(failure){
		throw new OccupancyGridAccessException(message);
	}
	return true;
}

};


