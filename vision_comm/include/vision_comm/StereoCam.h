/*
 * StereoCam.h
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#ifndef STEREOCAM_H_
#define STEREOCAM_H_

#include "vision_comm/IOImages.h"

class StereoCam: public IOImages {
	cv::VideoCapture 		cam1_;
	cv::VideoCapture 		cam2_;
	cv::Mat *				frame2_;

public:
	StereoCam(int camno1=1, int camno2=2);
	bool check();
	cv::Mat* getNextFrame();
	void getNextStereoFrame(cv::Mat lframe, cv::Mat rframe);
	virtual ~StereoCam();
	void showImage(std::string wname);
};

#endif /* STEREOCAM_H_ */
