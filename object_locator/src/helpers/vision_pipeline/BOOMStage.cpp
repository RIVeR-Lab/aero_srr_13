/*
 * BOOMStage.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: srr
 */

#include <object_locator/vision_pipeline/BOOMStage.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(object_locator, BOOMStage, object_locator::BOOMStage,
		nodelet::Nodelet)

namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;
using namespace cv;
using namespace std;

void BOOMStage::onInit()
{
	loadParams();
	registerTopics();
}

void BOOMStage::loadParams()
{
	this->input_topic_="sync_stage/stereo_pair";
	this->output_topic_="boom_stage/direction";
	this->getPrivateNodeHandle().getParam(this->input_topic_,this->input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,this->output_topic_);
	load_=imread("/home/srr/ObjectDetectionData/samplesOutsideDownscaled.jpg", CV_LOAD_IMAGE_COLOR);
	NODELET_INFO_STREAM("img height =" << load_.cols << "\n" << "img width =" << load_.rows);
}

void BOOMStage::registerTopics()
{
	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,2,&BOOMStage::boomImageCb,this);
	//this->disp_image_pub_ = this->getNodeHandle().advertise<object_locator::SyncImageMsg>(this->output_topic_,2);
}

void BOOMStage::boomImageCb(const object_locator::SyncImageMsgConstPtr& msg)
{
	Mat_t normImage;
	grassRemove(msg->left_image, normImage);
	blobIdentify(normImage);

}

void BOOMStage::grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage)
{
	cv_bridge::CvImagePtr img;
	Mat_t src = load_;
	try {
		img = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
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
	for(int y = 0; y<src.cols; y++)
	{
		for(int x = 0; x<src.rows; x++)
		{
			RGB = src.at<cv::Vec3b>(x,y);
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
//			normImg_.at<cv::Vec3b>(x,y) = nRGB;
			browness = nR/nG;
			whiteness = sumRGB/756;
			if((nG > .38) || ((std::abs(browness - 1) < .2) && (whiteness < .9)))
			{
			    normImage.at<cv::Vec3b>(x,y) = ZeroV;
			}
			else
			{
				normImage.at<cv::Vec3b>(x,y) = Black;
			}
		}
	}
}

void BOOMStage::blobIdentify(Mat_t& img)
{
	NODELET_INFO_STREAM("IN BLOB IDENTIFY");
	Mat_t med, src_gray,normImg;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	 medianBlur(img,med, 11);
	 cvtColor(normImg, src_gray, CV_BGR2GRAY);

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
	 	  if(flag[i] != 1)
	 	  {
	        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
	        circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
	        cout << "Center of object[" << i << "]" << "= "<< center[i].x<<","<< center[i].y << endl;
	 	  }
	      }

	   /// Show in a window
	   namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	   imshow( "Contours", drawing );
		std::stringstream s;
		s << "/home/srr/ObjectDetectionData/blob/0.png";
		cv::imwrite(s.str(), drawing);
}

void BOOMStage::generateMsg()
{

}
