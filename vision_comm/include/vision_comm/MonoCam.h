/*
 * MonoCam.h
 *
 *  Created on: Jan 25, 2013
 *      Author: bpwiselybabu
 */

#ifndef MONOCAM_H_
#define MONOCAM_H_

#include "IOImages.h"

class MonoCam: public IOImages {
	cv::VideoCapture cam_;
public:
	MonoCam(int camno);
	MonoCam();
	bool check();
	cv::Mat* getNextFrame();
	virtual ~MonoCam();
};

#endif /* MONOCAM_H_ */
