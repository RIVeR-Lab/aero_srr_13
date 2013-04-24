/*
 * IOImages.h
 *
 *  Created on: Jan 24, 2013
 *      Author: bpwiselybabu
 */

#ifndef IOIMAGES_H_
#define IOIMAGES_H_

#include "vision_comm/main.h"


class IOImages {

	timeval		clock_;
	int 		fps_frame_count_;
	double 		fps_total_time_;

protected:
	int 		frame_no_;
	double 		fps_;
	cv::Mat *	frame_;
	std::string file_;
	int			line_no_;
	cv::Mat		intrinsic_;

public:
	IOImages();
	virtual cv::Mat* getNextFrame()=0;
	double getfps();
	void startTime();
	double endTime();
	void addText(std::string txt);
	void showImage(std::string wname);
	void writeImage(std::string fname);
	cv::Mat intrinsic()
        {
		return(intrinsic_);
	}
	virtual ~IOImages();
};

#endif /* IOIMAGES_H_ */
