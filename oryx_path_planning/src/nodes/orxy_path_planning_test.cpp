/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include "Tentacles.h"

void printSpeedSet(int xDim, int yDim, double resoltuion, oryx_path_planning::SpeedSet& speedSet);

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	ros::NodeHandle nh;

	ROS_INFO("Testing Tentacle Generator...");
	int xDim=25;
	int yDim=xDim;
	double resolution = .25;
	int numTent= 23;
	double minSpeed = .1;
	double maxSpeed = 1.5;
	int numSpeedSet = 5;
	double expFact = 1.15;
	oryx_path_planning::TentacleGenerator generator(minSpeed, maxSpeed, numSpeedSet, numTent, expFact, resolution, xDim, yDim);
	ROS_INFO("Tentacles Generated. Printing Speed Sets...");
	for(int s=0; s<numSpeedSet; s++){
		printSpeedSet(xDim, yDim, resolution, generator.getSpeedSet(s));
	}

	/*	tf::Point test1;
	tf::Point test2;

	test1.a = 1;
	test1.b = 1;
	test2.a = 3;
	test2.b = 1;

	ROS_INFO("Answer to if Test1==Test2 is %s", (test1==test2)?"TRUE":"FALSE");
	test1 = test2;
	ROS_INFO("Answer to if Test1==Test2 is %s", (test1==test2)?"TRUE":"FALSE");*/

	return 0;
}


void printSpeedSet(int xDim, int yDim, double resolution, oryx_path_planning::SpeedSet& speedSet){
	int xSize = xDim/resolution;
	int ySize = (yDim/resolution)*2;
	int numTent = speedSet.getNumTentacle();
	std::vector<std::string> occGrid(xSize, std::string(ySize, ' '));

	for(unsigned int t=0; t<numTent; t++){
		//if(t!=numTent/2){
		oryx_path_planning::Tentacle tentacle = speedSet.getTentacle(t);
		for(unsigned int p=0; p<tentacle.getPoints().size(); p++){
			tf::Point point = tentacle.getPoints().at(p);
			//ROS_INFO("Got Point at <%f, %f>", point.getX(), point.getY());
			point.setY(point.getY()+ySize/2);
			//ROS_INFO("Placing Point at <%f, %f>", point.getX(), point.getY());
			occGrid.at(oryx_path_planning::roundToGrid(point.getX(), resolution)).replace(oryx_path_planning::roundToGrid(point.getY(), resolution),1,"T");
		}
		//}
	}
	std::string output;

	for(int i=0;i<occGrid.size(); i++){
		output+=occGrid.at(i);
		output+="\r\n";
	}

	ROS_INFO("\n%s", output.c_str());
}
