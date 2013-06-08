/*
 * kalman.h
 *
 *  Created on: Aug 19, 2012
 *      Author: benzun
 */

#ifndef KALMAN_H_
#define KALMAN_H_
#include <stdio.h>
#include <iostream>

#include <time.h>
#include <math.h>
#include <fstream>
#include <string>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class kalman {
	Mat A_,B_,H_,Q_,R_;
	Rect rect_,prect_;
	Mat X_;
	Mat P_;
	Mat prX_,prP_;
	Mat K_,Z_;
	bool track_,new_;
	int failcount_;
	int ctr_;
public:

	kalman();
	void getMeasurment(vector <Rect> results);
	void calcPriori();
	void calcGain();
	cv::Rect track();
	void calcCorrection();
	void drawResult(Mat *frame);
	virtual ~kalman();
};

#endif /* KALMAN_H_ */
