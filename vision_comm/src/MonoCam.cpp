/*
 * MonoCam.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/MonoCam.h"
using namespace cv;
using namespace std;

MonoCam::MonoCam(int no)
{
	char num[3];
	sprintf(num,"%d",no);
	cam_=VideoCapture(no); // open the default camera
	file_="CAM "+string(num);
	frame_=new Mat();
}
MonoCam::MonoCam()
{
	cam_=VideoCapture(0); // open the default camera
	file_="CAM 0";
	frame_no_=0;
	fps_=0;
	line_no_=0;
	frame_=new Mat();
}
bool MonoCam::check()
{
	if(!cam_.isOpened())  // check if we succeeded
	    return false;
	return true;
}
cv::Mat* MonoCam::getNextFrame()
{
	cam_>>*frame_;
	cout<<"TIME STAMP: "<<cam_.get(CV_CAP_PROP_POS_FRAMES)<<endl;
	line_no_=0;
	frame_no_++;

	return(frame_);
}
MonoCam::~MonoCam() {
	// TODO Auto-generated destructor stub
}
