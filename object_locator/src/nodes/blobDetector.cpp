/*
 * blobDetector.cpp
 *
 *  Created on: Apr 17, 2013
 *      Author: srr
 */
#include "object_locator/typedefinitions.h"


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	Mat image;
	vector<Vec4i> hierarchy;
	image = imread("/home/srr/ObjectDetectionData/Filtered_samples.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	vector<vector<Point> > contours;
	Mat dst= Mat::zeros(image.rows, image.cols, CV_8UC3);
	int mode =  CV_RETR_CCOMP;
	int method = CV_CHAIN_APPROX_SIMPLE;;
	findContours(image,contours,hierarchy,mode, method);
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
    }
    namedWindow( "Components", 1 );
    imshow( "Components", dst );
    waitKey(0);
}

