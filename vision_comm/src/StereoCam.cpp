/*
 * StereoCam.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/StereoCam.h"
using namespace cv;
using namespace std;
StereoCam::StereoCam(int camno1, int camno2)
{
	cam1_=VideoCapture(camno1); // open the default camera
	cam2_=VideoCapture(camno2);
	file_="Stero CAM";
	frame_=new Mat();
	frame2_=new Mat();
}
Mat* StereoCam::getNextFrame()
{
	cout<<"StereoCam does not support this function use getNextStereoFrame()"<<endl;
	return(NULL);
}
bool StereoCam::check()
{
	if(cam1_.isOpened()&&cam2_.isOpened())  // check if we succeeded
	    return true;
	return false;
}
void StereoCam::getNextStereoFrame(Mat lframe, Mat rframe)
{
	/*
	 * TOFIX: If the cameras have the same bus they fail. Need to find a solution
	 *       And the current implementation leads to two copies in the memory at differnt
	 *       addresses.
	 */
	cam1_>>*frame_;
	cam2_>>*frame2_;
	line_no_=0;
	frame_no_++;
	lframe=*frame_;
	cout<<frame_<<" : "<<&lframe<<endl;
	rframe=*frame2_;
	cout<<frame2_<<" : "<<&rframe<<endl;
}
void StereoCam::showImage(string wname)
{
	char label[500];
	sprintf(label,"File: %s",file_.c_str());
	addText(label);
	sprintf(label,"Frame No: %d",frame_no_);
	addText(label);
	sprintf(label,"FPS: %f",fps_);
	addText(label);
	imshow(wname+string("_left"),*frame_);
	imshow(wname+string("_right"),*frame2_);
}
StereoCam::~StereoCam() {
	// TODO Auto-generated destructor stub
}
