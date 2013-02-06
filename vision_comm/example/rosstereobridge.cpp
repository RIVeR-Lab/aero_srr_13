/*
 * rosstereobridge.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/RosStereoBridge.h"
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosstereocamera");
	Mat *frame;
	vector<Mat*> image_pair;
	double fps;

	if(argc!=3)
	{
		cout<<"Useage ./"<<argv[0]<<" <input_topic_base> <output>"<<endl;
		return -1;
	}

	RosStereoBridge rosstereocamera(argv[1]);
	ros::Rate loop(10);
	while(ros::ok())
	{
		rosstereocamera.startTime();
		image_pair=rosstereocamera.getNextStereoImage();
		frame=rosstereocamera.getNextFrame();
		if(!frame)
		{
			fps=rosstereocamera.endTime();
			rosstereocamera.showImage("Result");
			rosstereocamera.showPair("Result");
		}
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
