/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include "OryxPathPlanning.h"

void printSpeedSet(int xDim, int yDim, double resoltuion, oryx_path_planning::SpeedSet& speedSet);

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	ros::NodeHandle nh("~");


	ROS_INFO("Testing Tentacle Generator...");
	int xDim=50;
	int yDim=xDim/2;
	double resolution = .25;
	int numTent= 81;
	double minSpeed = .1;
	double maxSpeed = 1.5;
	int numSpeedSet = 16;
	double expFact = 1.15;
	int firstSpeedSet=0;
	int lastSpeedSet=numSpeedSet;
	oryx_path_planning::TentacleGenerator generator(minSpeed, maxSpeed, numSpeedSet, numTent, expFact, resolution, xDim, yDim);
	ROS_INFO("Tentacles Generated. Printing Speed Sets...");
	if(!nh.getParam("first_speed_set", firstSpeedSet))ROS_WARN("First Speed Set Not Set! Using Default %d", firstSpeedSet);
	if(!nh.getParam("last_speed_set", lastSpeedSet))ROS_WARN("Last Speed Set Not Set! Using Default %d", lastSpeedSet);

	for(int s=firstSpeedSet; s<lastSpeedSet; s++){
		printSpeedSet(xDim, yDim, resolution, generator.getSpeedSet(s));
	}

	ROS_INFO("Speed Sets Printed!");
	ROS_INFO("Testing Tentacle Traversal");
	try{
		oryx_path_planning::Tentacle::TentacleTraverser traverser(generator.getTentacle(firstSpeedSet, 0));
		while(traverser.hasNext()){
			tf::Point travPoint = traverser.next();
			ROS_INFO("Traversed Point <%f, %f>, total length = %f", travPoint.getX(), travPoint.getY(), traverser.lengthTraversed());
		}
	}catch (std::exception& e){
		ROS_ERROR("%s",e.what());
	}

	ROS_INFO("Test Some Exceptions:");

	oryx_path_planning::SpeedSetAccessException testE1(100);
	oryx_path_planning::TentacleAccessException testE2(1, 100, *(new std::string("Testing of Chained Exceptions")), testE1);

	try{
		throw testE2;
	}catch (std::exception& e){
		ROS_ERROR("%s", e.what());
	}

	ROS_INFO("Testing Occupancy Grid...");
	oryx_path_planning::OccupancyGrid testGrid(xDim,yDim*2, resolution, oryx_path_planning::UNKNOWN);
	ROS_INFO("Occupancy Grid Built");
	try{
		ROS_INFO("\n%s", testGrid.toString(2,0).get()->c_str());
	}catch(std::exception& e){
		ROS_ERROR(e.what());
	}
	ROS_INFO("Placing some test points on the grid");
	double y = 0;
	for(double x=0; x<xDim; x+=resolution){
		y= 1.0/2.0*x;
		testGrid.setPointTrait(x, y, 0, oryx_path_planning::FREE_LOW_COST);
	}
	try{
		ROS_INFO("\n%s", testGrid.toString(2,0).get()->c_str());
	}catch(std::exception& e){
		ROS_ERROR(e.what());
	}
	return 0;
}


void printSpeedSet(int xDim, int yDim, double resolution, oryx_path_planning::SpeedSet& speedSet){
	int xSize = xDim/resolution;
	int ySize = (yDim/resolution)*2;
	int numTent = speedSet.getNumTentacle();
	std::vector<std::string> occGrid(xSize+1, std::string(ySize+1, '_'));
	ROS_INFO("Occupancy Gird Generated");
	for(int t=0; t<numTent; t++){
		oryx_path_planning::Tentacle tentacle = speedSet.getTentacle(t);
		//ROS_INFO("Got Tentacle %d", t);
		for(unsigned int p=0; p<tentacle.getPoints().size(); p++){
			tf::Point point(tentacle.getPoints().at(p));
			//ROS_INFO("Got Point at <%f, %f>", point.getX(), point.getY());
			point.setX(oryx_path_planning::roundToGrid(point.getX(), resolution));
			point.setY(oryx_path_planning::roundToGrid(point.getY(), resolution)+(ySize/2));
			//ROS_INFO("Placing Point at <%f, %f>", point.getX(), point.getY());
			occGrid.at(point.getX()).replace(point.getY(),1,"T");
		}
		//ROS_INFO("Done with Tentacle %d", t);
	}
	std::string output;

	for(unsigned int i=0;i<occGrid.size(); i++){
		output+=occGrid.at(i);
		output+="\r\n";
	}

	ROS_INFO("\n%s", output.c_str());
}
