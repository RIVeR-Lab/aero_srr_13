/*
 * blobBox.cpp
 *
 *  Created on: Apr 17, 2013
 *      Author: srr
 */




#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray; Mat sec;Mat med; Mat normImg;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

/// Function header
void thresh_callback();
void normAndFilter();

/** @function main */
int main( int argc, char** argv )
{
  /// Load source image and convert it to gray
//  src =  imread("/home/srr/ObjectDetectionData/Filtered_samples.jpg", CV_LOAD_IMAGE_GRAYSCALE);
//  src_gray = src;
	clock_t start;
	start = clock();
	cout << "Start time : " << (double)((clock() - start)) << endl;
  sec =  imread("/home/srr/ObjectDetectionData/samplesOutsideDownscaled.jpg", CV_LOAD_IMAGE_COLOR);

  medianBlur(sec,med, 11);
//  char* source_window = "Source";
//  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
//  imshow( source_window, med);
  normImg = med;
  normAndFilter();
  cvtColor(normImg, src_gray, CV_BGR2GRAY);


//  char* filtered_window = "Filtered";
//  namedWindow( filtered_window, CV_WINDOW_AUTOSIZE );
//  imshow( filtered_window, normImg);

  thresh_callback();

  waitKey(0);
  cout << "End time : " << (double)((clock() - start) ) << endl;
  return(0);
}

void normAndFilter()
{

	double R,G,B,sumRGB, nR,nG,nB, browness,whiteness;
	Vec3b RGB,nRGB,ZeroV, Black;
	nRGB[0] = 0;
	nRGB[1] = 0;
	nRGB[2] = 0;
	ZeroV[0] = 0;
	ZeroV[1] = 0;
	ZeroV[2] = 0;
	Black[0] = 255;
	Black[1] = 255;
	Black[2] = 255;
	for(int y = 0; y<med.cols; y++)
	{
		for(int x = 0; x<med.rows; x++)
		{
			RGB = med.at<cv::Vec3b>(x,y);
			R = RGB[2];
			G = RGB[1];
			B = RGB[0];
			sumRGB = R + G + B;
			nR = R/sumRGB;
			nG = G/sumRGB;
			nB = B/sumRGB;
//			nRGB[0] = nR;
//			nRGB[1] = nG;
//			nRGB[2] = nB;
//			normImg.at<cv::Vec3b>(x,y) = nRGB;
			browness = nR/nG;
			whiteness = sumRGB/756;
			if((nG > .38) || ((std::abs(browness - 1) < .2) && (whiteness < .9)))
			{
			    normImg.at<cv::Vec3b>(x,y) = ZeroV;
			}
			else
			{
				normImg.at<cv::Vec3b>(x,y) = Black;
			}
		}
	}

}

/** @function thresh_callback */
void thresh_callback()
{
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;


  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );
  int flag[contours.size()];
  for( int i = 0; i < contours.size(); i++ )
	  {

	  approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
 	  if(radius[i] > src_gray.cols/3)
 	     	   flag[i] = 1;
 	  else
 		  flag[i] = 0;
     }


  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
	  if(flag[i] !=1)
	  {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
       cout << "Center of object[" << i << "]" << "= "<< center[i].x<<","<< center[i].y << endl;
	  }
     }

  /// Show in a window
//  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//  imshow( "Contours", drawing );
}
