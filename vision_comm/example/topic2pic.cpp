/*
 * rosbridge.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/RosBridge.h"
#include <stdlib.h>
#define BASE_ADDR "/home/bpwiselybabu/imageset/"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roscamera");
	Mat *frame;
	double fps;

	if(argc!=3)
	{
		cout<<"Useage ./roscam <input> <folder_name>"<<endl;
		return -1;
	}

	RosBridge roscamera(argv[1],frame);
	ros::Rate loop(10);
	int i=0;
	char loc[200];
	while(ros::ok())
	{
		roscamera.startTime();
		frame=roscamera.getNextFrame();
		if(frame!=NULL)
		{
			//cout<<"Entered";
			sprintf(loc,"%s/%s/Capture%04d.png",string(BASE_ADDR).c_str(),argv[2],i);
			i++;
			imwrite(loc,*frame);
			roscamera.showImage("Result");
			fps=roscamera.endTime();
		}
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
