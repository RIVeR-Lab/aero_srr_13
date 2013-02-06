/*
 * pubtopic.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/RosBridge.h"
#include "vision_comm/MonoCam.h"
#include <stdlib.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pubcam");
	Mat *frame;
	double fps;

	if(argc!=3)
	{
		cout<<"Usage: ./pubcam <camera_no> <output_topic>"<<endl;
		return -1;
	}


	MonoCam camera(atoi(argv[1]));
    if(!camera.check())
    {
    	cout<<"Camera "<<argv[1]<<" not found"<<endl;
    	return -1;
    }
    frame=camera.getNextFrame();

	RosBridge roscamera(argv[2],frame);

	ros::Rate loop(10);
	char ch;
	while(ros::ok()&&ch!='q')
	{
		camera.startTime();
		frame=camera.getNextFrame();
		fps=camera.endTime();
		frame=roscamera.getNextFrame();
		camera.showImage("Result");
		roscamera.publishFrame();
		ch=waitKey(1);
		ros::spin();
		loop.sleep();
	}
	return 0;
}
