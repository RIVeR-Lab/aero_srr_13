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

/**
 * @todo Currently only 2D
 */
OccupancyGrid::OccupancyGrid(double xDim, double yDim, double zDim, double resolution, PointTrait_t seedTrait):
		occGrid(new pcl::PointCloud<pcl::PointXYZRGBA>(roundToGrid(xDim, resolution),roundToGrid(yDim, resolution))){
	this->xDim	= roundToFrac(xDim, resolution);
	this->yDim	= roundToFrac(yDim, resolution);
	this->zDim	= roundToFrac(zDim, resolution);
	this->res	= resolution;
	//Initialize the grid
	for(unsigned int x=0; x<this->occGrid.get()->width; x++){
		for(unsigned int y=0; y<this->occGrid.get()->height; y++){
			//Initialize the point
			pcl::PointXYZRGBA& point = this->occGrid.get()->at(x,y);
			point.x = gridToReal(x, this->res);
			point.y = gridToReal(y, this->res);
			point.rgba = seedTrait;
		}
	}
}

OccupancyGrid::~OccupancyGrid(){}

/**
 * @todo Currently only 2D
 */
PointTrait OccupancyGrid::getPointTrait(double x, double y, double z)throw(OccupancyGridAccessException){
	boundsCheck(x,y,z);
	return static_cast<oryx_path_planning::PointTrait>(this->occGrid.get()->at(roundToGrid(x, this->res), roundToGrid(y, this->res)).rgba);
}

/**
 * @todo Currently only 2D
 */
bool OccupancyGrid::setPointTrait(double x, double y, double z, oryx_path_planning::PointTrait_t trait)throw(OccupancyGridAccessException){
	boundsCheck(x,y,z);
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

/**
 * @todo Currently only 2D
 */
boost::shared_ptr<std::string> OccupancyGrid::toString(int sliceAxis, double slice){
	boost::shared_ptr<std::string> output(new std::string(""));
	std::vector<std::string> occGrid(this->occGrid.get()->width, std::string(this->occGrid.get()->height, ' '));

	char value[2] ={0,'\0'};
	for(unsigned int x=0; x<this->occGrid.get()->width; x++){
		for(unsigned int y=0; y<this->occGrid.get()->height; y++){
			switch(this->occGrid.get()->at(x,y).rgba){
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
	return failure;
}

};


