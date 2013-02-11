/*
 * IOImages.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/IOImages.h"
using namespace std;
using namespace cv;
IOImages::IOImages() {
	fps_frame_count_=0;
	fps_total_time_=0;
	frame_no_=0;
	fps_=0.0;
	line_no_=0;
}
void IOImages::startTime()
{
	gettimeofday(&clock_,NULL);
}
double IOImages::endTime()
{
	timeval endtime;
	gettimeofday(&endtime,NULL);
	long seconds  = endtime.tv_sec  - clock_.tv_sec;
	long useconds = endtime.tv_usec - clock_.tv_usec;

    double t = ((seconds) + useconds*0.000001);
	fps_total_time_ += t;
	fps_frame_count_++;
	if(fps_frame_count_==10)
	{
		fps_=10/fps_total_time_;
		fps_frame_count_=0;
		fps_total_time_=0;
	}
	return(fps_);
}
double IOImages::getfps()
{
	return fps_;
}
void IOImages::addText(string txt)
{
	char label[500];
	sprintf(label,"%s",txt.c_str());
	putText(*frame_,label,Point(10,30+line_no_*20),FONT_HERSHEY_PLAIN,1,Scalar(255,255,255),1);
	line_no_+=1;
}
void IOImages::showImage(string wname)
{
	//print the file name and the frame number when you show output;
	char label[500];
	sprintf(label,"File: %s",file_.c_str());
	addText(label);
	sprintf(label,"Frame No: %d",frame_no_);
    addText(label);
	sprintf(label,"FPS: %f",fps_);
	addText(label);
	imshow(wname.c_str(),*frame_);
	line_no_=0;
}
void IOImages::writeImage(string fname)
{
	char name[500];
	sprintf(name,"%s/%05d.jpg",fname.c_str(),frame_no_);
	imwrite(name,*frame_);
}

IOImages::~IOImages() {
	// TODO Auto-generated destructor stub
}
