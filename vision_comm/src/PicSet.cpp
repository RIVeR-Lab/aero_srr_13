/*
 * PicSet.cpp
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#include "vision_comm/PicSet.h"
using namespace std;
using namespace cv;
PicSet::PicSet(int begin, int end, string location, string prefix ,string ext)
{
	start_=begin;
	end_=end;
	location_=location;
	prefix_=prefix;
	ext_=ext;
	file_=prefix+ext;
	frame_=new Mat();

}
cv::Mat* PicSet::getNextFrame()
{
	char file[200];
	sprintf(file,"%s%04d",prefix_.c_str(),start_+frame_no_);
	file_=string(file);
	*frame_=imread(location_+file_+ext_);
	frame_no_++;
	line_no_=0;
	return(frame_);
}
PicSet::~PicSet() {
	// TODO Auto-generated destructor stub
}
