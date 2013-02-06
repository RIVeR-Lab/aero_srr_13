/*
 * rosbridge.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/RosBridge.h"
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roscamera");
	Mat *frame;
	double fps;

	if(argc!=3)
	{
		cout<<"Useage ./roscam <input> <output>"<<endl;
		return -1;
	}

	RosBridge roscamera(argv[1],argv[2]);
	ros::Rate loop(10);
	while(ros::ok())
	{
		roscamera.startTime();
		frame=roscamera.getNextFrame();
		if(frame!=NULL)
		{
			//cout<<"Entered";
			fps=roscamera.endTime();
			roscamera.showImage("Result");
			roscamera.publishFrame();
		}
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
