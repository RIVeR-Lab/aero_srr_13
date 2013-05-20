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

void BOOMStage::onInit() {
	loadParams();
	registerTopics();
}

void BOOMStage::loadParams() {
	this->input_topic_ = "stereo_camera/left/image_raw";
	this->output_topic_ = "boom_stage/direction";
	this->getPrivateNodeHandle().getParam(this->input_topic_,
			this->input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,
			this->output_topic_);
	this->it_ = new image_transport::ImageTransport(this->getNodeHandle());
	this->HORIZON_TOP_ = 200;
	this->HORIZON_BTM_ = 730;
	this->ZeroV[0] = 0;
	this->ZeroV[1] = 0;
	this->ZeroV[2] = 0;

	this->White[0] = 255;
	this->White[1] = 255;
	this->White[2] = 255;
//	load_=imread("/home/srr/ObjectDetectionData/samplesOutsideDownscaled.jpg", CV_LOAD_IMAGE_COLOR);
//	NODELET_INFO_STREAM("img height =" << load_.cols << "\n" << "img width =" << load_.rows);
}

void BOOMStage::registerTopics() {
	this->image_left_ = it_->subscribeCamera(this->input_topic_, 2,
			&BOOMStage::boomImageCb, this);
//	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,2,&BOOMStage::boomImageCb,this);
	//this->disp_image_pub_ = this->getNodeHandle().advertise<object_locator::SyncImageMsg>(this->output_topic_,2);
}

void BOOMStage::boomImageCb(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& info) {
	cv_bridge::CvImagePtr img;
	NODELET_INFO_STREAM("In Boom Image CB");
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat_t src = img->image;
	Mat_t normImage,mask;
	grassRemove(*msg, normImage,mask);
//	fenceCheckerStd(normImage);
	fillHoles(mask);
	blobIdentify(normImage,mask);

}

void BOOMStage::grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage, Mat_t& maskt) {
	NODELET_INFO_STREAM("IN BLOB GRASS REMOVE");
	cv_bridge::CvImagePtr img;
//	Mat_t src = load_;
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat_t src = img->image;
	Mat_t psrc = src;
	double R, G, B, sumRGB, nR, nG, nB, browness, whiteness;
	Vec3b RGB, nRGB;
	Mat_t norma = img->image;
	Mat_t mask = src;


	nRGB[0] = 0;
	nRGB[1] = 0;
	nRGB[2] = 0;

	GaussianBlur(src,src,Size(9,9),2,8,0);
	for (int y = 0; y < src.cols; y++) {
		for (int x = 0; x < src.rows; x++) {
			RGB = src.at<cv::Vec3b>(x, y); //x is y, y is x
			B = RGB[2];
			G = RGB[1];
			R = RGB[0];
			sumRGB = R + G + B;
			nR = R / sumRGB;
			nG = G / sumRGB;
			nB = B / sumRGB;
//			nRGB[0] = nR;
//			nRGB[1] = nG;
//			nRGB[2] = nB;
//			normImg_.at<cv::Vec3b>(x,y) = nRGB;
			browness = nR / nG;
			whiteness = sumRGB / 756;
			if ((nG > .33)
					|| ((std::abs(browness - 1) < .2) && (whiteness < .9))) {
				norma.at<cv::Vec3b>(x, y) = ZeroV;
				mask.at<cv::Vec3b>(x, y) = White;
			} else {
				norma.at<cv::Vec3b>(x, y) = White;
				mask.at<cv::Vec3b>(x, y) = ZeroV;
			}
			if ((x == 278) && (y == 803)) {
				NODELET_WARN_STREAM(
						"nR = " << nR << std::endl << "nG =" << nG << std::endl << "nB =" << nB);

			}
			if ((nR > nG) && (nG > nB) && (nR > .40)) {
//				NODELET_WARN_STREAM("Fence Detected");

			}
		}
//		HORIZON_ = Fence_;
	}
	for(int k = 0; k< HORIZON_TOP_; k++)
		{
		for(int m = 0;m<norma.cols; m++)
		{
			norma.at<cv::Vec3b>(k, m) = ZeroV;
		}
		}

	dilate(mask,mask, cv::Mat(4,4,CV_8UC1));
	erode(mask,mask, cv::Mat(30,30,CV_8UC1));
	medianBlur(mask,mask,15);
	cvtColor(mask,mask,CV_RGB2GRAY);
//	for(int l = 0; l< norma.cols; l++)
//	{
//		mask.at<cv::Vec3b>(0, l) = ZeroV;
//		mask.at<cv::Vec3b>(norma.rows-1, l) = ZeroV;
//	}
//
//	for(int k = 0; k< norma.rows; k++)
//		{
//			mask.at<cv::Vec3b>(k, 0) = ZeroV;
//			mask.at<cv::Vec3b>(k, norma.cols-1) = ZeroV;
//		}

	maskt = mask;

	normImage = norma;
	cv::line(mask, Point2d(0, HORIZON_TOP_), Point2d(norma.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));
	cv::line(norma, Point2d(0, HORIZON_BTM_), Point2d(norma.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));
//	   namedWindow( "b&w", CV_WINDOW_AUTOSIZE );
	imshow("mask", mask);
	waitKey(3);

}

void BOOMStage::blobIdentify(Mat_t& img, Mat_t& mask) {
	NODELET_INFO_STREAM("IN BLOB IDENTIFY");
	Mat_t med, src_gray, normImg;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	medianBlur(img, med, 11);

	normImg = med;
	for(int k = 0; k< normImg.rows; k++)
		{
		for(int m = 0;m<normImg.cols; m++)
		{
			if(mask.at<cv::Vec3b>(k, m) == ZeroV)
				normImg.at<cv::Vec3b>(k, m)= ZeroV;
		}
		}
	cvtColor(normImg, src_gray, CV_BGR2GRAY);



	/// Detect edges using Threshold
	threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);

	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());
	int flag[contours.size()];
	for (int i = 0; i < contours.size(); i++) {

		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
		if (radius[i] > src_gray.cols / 3)
			flag[i] = 1;
		else
			flag[i] = 0;
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		if (flag[i] != 1) {
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
					rng.uniform(0, 255));
			drawContours(drawing, contours_poly, i, color, 1, 8,
					vector<Vec4i>(), 0, Point());
			if ((int) radius[i] > 35 && (int) radius[i] < 100) {
				rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2,
						8, 0);
				circle(drawing, center[i], (int) radius[i], color, 2, 8, 0);
				cout << "Center of object[" << i << "]" << "= " << center[i].x
						<< "," << center[i].y << endl;
			}
		}
	}

	/// Show in a window
	cv::line(drawing, Point2d(0, HORIZON_TOP_), Point2d(drawing.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));
	cv::line(drawing, Point2d(0, HORIZON_BTM_), Point2d(drawing.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);

	cv::waitKey(3);

//		std::stringstream s;
//		s << "/home/srr/ObjectDetectionData/blob/0.png";
//		cv::imwrite(s.str(), drawing);
}

void BOOMStage::fillHoles(Mat_t& img) {
	NODELET_INFO_STREAM("IN FILLHOLES IDENTIFY");
	Mat_t med, src_gray, normImg;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

src_gray  = img;

	/// Detect edges using Threshold
	threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);

	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());
	int flag[contours.size()];
	for (int i = 0; i < contours.size(); i++) {

		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
		if (radius[i] > src_gray.cols / 3)
			flag[i] = 1;
		else
			flag[i] = 0;
	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		if (flag[i] != 1) {
			if(radius[i] <100)
				rectangle(img, boundRect[i].tl(), boundRect[i].br(), 255, CV_FILLED,
										8, 0);

		}
	}

	/// Show in a window
	cv::line(drawing, Point2d(0, HORIZON_TOP_), Point2d(drawing.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));
	cv::line(drawing, Point2d(0, HORIZON_BTM_), Point2d(drawing.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));
	imshow("filled", img);


	cv::waitKey(3);

//		std::stringstream s;
//		s << "/home/srr/ObjectDetectionData/blob/0.png";
//		cv::imwrite(s.str(), drawing);
}

void BOOMStage::generateMsg() {

}
