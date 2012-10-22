/**
 * @file	OccupancyGrid.cpp
 * @date	Oct 22, 2012
 * @author	Adam Panzica
 * @brief	Contains the implementations for OccupancyGrid.h
 */

#include "OccupancyGrid.h"

using namespace oryx_path_planning;

namespace oryx_path_planning{

OccupancyGrid::OccupancyGrid():
		occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>()){
	this->xDim	= 0;
	this->yDim	= 0;
	this->zDim	= 0;
	this->res	= 0;
}

OccupancyGrid::OccupancyGrid(oryx_path_planning::OccupancyGrid& grid){
	this->xDim	= grid.xDim;
	this->yDim	= grid.yDim;
	this->zDim	= grid.zDim;
	this->res	= grid.res;
	this->occGrid = grid.getGrid();
};

OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, PointTrait seetTrait):
		occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>(roundToGrid(xDim, resolution),roundToGrid(yDim, resolution))){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= roundToFrac(zDim, resolution);
	this->res	= resolution;
}

OccupancyGrid::~OccupancyGrid(){}

PointTrait OccupancyGrid::getPointTrait(double x, double y, double z){
	return static_cast<oryx_path_planning::PointTrait>(this->occGrid.get()->at(roundToGrid(x, this->res), roundToGrid(y, this->res)).rgba);
}

bool OccupancyGrid::setPointTrait(double x, double y, double z, oryx_path_planning::PointTrait trait){
	this->occGrid.get()->at(roundToGrid(x, this->res), roundToGrid(y, this->res)).rgba=trait;
	return true;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > OccupancyGrid::getGrid(){
	return this->occGrid;
}

bool OccupancyGrid::generateMessage(sensor_msgs::PointCloud2& message){
	pcl::toROSMsg(*this->occGrid, message);
	return true;
}

boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, double slice){
	return boost::shared_ptr<std::string>(new std::string("Not Yet Implemented!"));
}
};


