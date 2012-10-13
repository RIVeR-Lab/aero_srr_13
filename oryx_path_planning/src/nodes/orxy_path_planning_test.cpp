/**
 * @file	orxy_path_planning_test.cpp
 * @date	Oct 11, 2012
 * @author	Adam Panzica
 * @brief	Simple test node
 */
#include <ros/ros.h>
#include "Tentacles.h"

void printSpeedSet(int xDim, int yDim, oryx_path_planning::SpeedSet& speedSet);

int main(int argc, char **argv) {
	ros::init(argc, argv, "oryx_path_planning_test");
	ros::NodeHandle nh;
	std::vector<double > speedSets;
	ROS_INFO("Testing Tentacle Generator...");
	for(int i=0; i<3; i++){
		speedSets.push_back(.01*(i+1));
	}
	int xDim=25;
	int yDim=xDim;
	oryx_path_planning::TentacleGenerator generator(15, 1.15, .25, xDim*.25, yDim*.25, speedSets);
	printSpeedSet(xDim, yDim, generator.getSpeedSet(0));

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


void printSpeedSet(int xDim, int yDim, oryx_path_planning::SpeedSet& speedSet){
	int xSize = xDim*2;
	int ySize = yDim;
	char occGrid[xSize][ySize+2];
	memset(occGrid, ' ', xSize*(ySize+2)*sizeof(char));

	//Fill in null newline and terminator to make creating strings easy
	for(int i=0; i<xSize; i++){
		occGrid[i][ySize]	= '\n';
		occGrid[i][ySize+1]	= '\0';
	}

	for(unsigned int t=0; t<3/*speedSet.getNumTentacle()*/; t++){
		oryx_path_planning::Tentacle tentacle = speedSet.getTentacle(t);
		for(unsigned int p=0; p<tentacle.getPoints().size(); p++){
			tf::Point point = tentacle.getPoints().at(p);
			//ROS_INFO("Got Point at <%f, %f>", point.getX(), point.getY());
			point.setY(point.getY()+ySize);
			//ROS_INFO("Placing Point at <%f, %f>", point.getX(), point.getY());
			occGrid[(int)point.getX()][(int)point.getY()] = 'T';
		}
	}
	std::string output;
	for(int x=0; x<xDim; x++){
		std::string line(occGrid[x]);
		//ROS_INFO(line.c_str());
		output.append(line);
	}

	ROS_INFO("\n%s", output.c_str());
}
