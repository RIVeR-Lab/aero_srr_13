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
	this->left_input_topic_ = "upper_stereo/left/image_raw";
	this->right_input_topic_ = "upper_stereo/right/image_raw";
	this->output_topic_ = "boom_stage/poses";
	this->getPrivateNodeHandle().getParam(this->left_input_topic_,
			this->left_input_topic_);
	this->getPrivateNodeHandle().getParam(this->right_input_topic_,
			this->right_input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,
			this->output_topic_);
	this->it_ = new image_transport::ImageTransport(this->getNodeHandle());
	this->HORIZON_TOP_ = 125;
	this->HORIZON_BTM_ = 730;
	this->ZeroV[0] = 0;
	this->ZeroV[1] = 0;
	this->ZeroV[2] = 0;

	this->White[0] = 255;
	this->White[1] = 255;
	this->White[2] = 255;
	this->got_left_ = false;
	this->got_right_ = false;

//	std::string thresh_dist("thresh_dist");
	thresh_dist_ = .5;
//	this->getPrivateNodeHandle().getParam(thresh_dist, thresh_dist_);

//	std::string growth_rate("growth_rate");
	growth_rate_ = .10;
//	this->getPrivateNodeHandle().getParam(growth_rate, growth_rate_);

//	std::string shrink_rate("shrink_rate");
	shrink_rate_ = .05;
//	this->getPrivateNodeHandle().getParam(shrink_rate, shrink_rate_);

//	std::string thresh_det("thresh_det");
	thresh_det_ = .5;
//	this->getPrivateNodeHandle().getParam(thresh_det, thresh_det_);
	this->watson_ = new DetectionManager(thresh_dist_, growth_rate_, shrink_rate_, thresh_det_);
//	load_=imread("/home/srr/ObjectDetectionData/samplesOutsideDownscaled.jpg", CV_LOAD_IMAGE_COLOR);
//	NODELET_INFO_STREAM("img height =" << load_.cols << "\n" << "img width =" << load_.rows);
}

void BOOMStage::registerTopics() {
	this->image_left_ = it_->subscribeCamera(this->left_input_topic_, 2,
			&BOOMStage::boomImageCbleft, this);
//	this->image_right_ = it_->subscribeCamera(this->right_input_topic_, 2,
//				&BOOMStage::boomImageCbright, this);
//	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,2,&BOOMStage::boomImageCb,this);
	this->pose_array_pub_ = this->getNodeHandle().advertise<geometry_msgs::PoseArray>(this->output_topic_,2);
}

void BOOMStage::boomImageCbleft(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& info) {
	cv_bridge::CvImagePtr img;
	NODELET_INFO_STREAM("In Boom Image CB");
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	left_image_ = img->image;
	left_info_ = *info;
	got_left_ = true;
//	computeDisparityCb();
	Mat_t src = img->image;
	Mat_t normImage,mask,finalMask;
	grassRemove(*msg, normImage);
	maskCreate(*msg,mask);
//	fenceCheckerStd(normImage);
//	fillHoles(mask);
	blobIdentify(normImage,mask, finalMask);
	showAlpha(img->image,finalMask);
	detectAnomalies(normImage,finalMask);

}
void BOOMStage::boomImageCbright(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& info) {
	cv_bridge::CvImagePtr img;
	NODELET_INFO_STREAM("In Boom Image CB");
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	right_image_ = img->image;
	right_info_ = *info;
	got_right_ = true;

}
void BOOMStage::computeDisparityCb() {
	if (got_left_ && got_right_) {
		computeDisparity();
		got_left_ = false;
		got_right_ = false;
	}
//	computeDisparity();
}
void BOOMStage::showAlpha(Mat_t& src, Mat_t& fMask)
{
	Mat_t overlay,temp;
	double alpha = .5;
	double beta = ( 1.0 - alpha );

	addWeighted(src,alpha,fMask,beta,0.0,overlay);
	imshow("overlay",overlay);
	waitKey(3);
}
void BOOMStage::grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage) {
	NODELET_INFO_STREAM("IN BLOB GRASS REMOVE");
	cv_bridge::CvImagePtr img;
//	Mat_t src = load_;
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat_t src(img->image);
	Mat_t psrc = src;
	double R, G, B, sumRGB, nR, nG, nB, browness, whiteness;
	Vec3b RGB, nRGB;
	Mat_t norma(img->image);



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
			if ((nB > .33)
					|| ((std::abs(browness - 1) < .2) && (whiteness < .9))) {
				norma.at<cv::Vec3b>(x, y) = ZeroV;

			}
			else {
				norma.at<cv::Vec3b>(x, y) = White;

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




	normImage = norma;

	cv::line(norma, Point2d(0, HORIZON_BTM_), Point2d(norma.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));
;

//
//imshow("norm", norma);
//waitKey(3);


}
void BOOMStage::maskCreate(const sensor_msgs::Image& msg, Mat_t& maskt) {
	NODELET_INFO_STREAM("IN BLOB GRASS REMOVE");
	cv_bridge::CvImagePtr img;
//	Mat_t src = load_;
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat_t src(img->image);
	Mat_t psrc = src;
	double R, G, B, sumRGB, nR, nG, nB, browness, whiteness;
	Vec3b RGB, nRGB;
	Mat_t norma(img->image);
	Mat_t mask(img->image);


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
				mask.at<cv::Vec3b>(x, y) = White;
			}
			else {
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
		for(int m = 0;m<mask.cols; m++)
		{
			mask.at<cv::Vec3b>(k, m) = ZeroV;
		}
		}
	dilate(mask,mask, cv::Mat(4,4,CV_8UC1));
	erode(mask,mask, cv::Mat(30,30,CV_8UC1));
	medianBlur(mask,mask,15);
	cvtColor(mask,mask,CV_RGB2GRAY);
	rectangle(mask, Point(0,0), Point(mask.cols,mask.rows), Scalar(0,0,0), 5,
							8, 0);

	maskt = mask;


	cv::line(mask, Point2d(0, HORIZON_TOP_), Point2d(mask.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));


	imshow("mask", mask);
	waitKey(3);

}

void BOOMStage::blobIdentify(const Mat_t& img, Mat_t& mask, Mat_t& final) {
	NODELET_INFO_STREAM("IN BLOB IDENTIFY");
	Mat_t src_gray(mask);
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;



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
	Mat finalMask = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++) {
		if (flag[i] != 1) {
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
					rng.uniform(0, 255));
			drawContours(finalMask, contours_poly, i, Scalar(255,255,255), CV_FILLED, 8,
					vector<Vec4i>(), 0, Point());
			if ((int) radius[i] > 200) {
				rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2,
						8, 0);
				circle(drawing, center[i], (int) radius[i], color, 2, 8, 0);

//				rectangle(finalMask, boundRect[i].tl(), boundRect[i].br(),Scalar(255,0,0,50) , CV_FILLED,8, 0);
			}
		}
	}

	Mat_t binaryMask, maskTemp;
	maskTemp = mask;
	Mat_t binaryFinal, finalMaskGray;
	Mat_t binaryMerged;
//	for(int k = 0; k< mask.rows; k++)
//		{
//		for(int m = 0;m <mask.cols; m++)
//		{
//			if(mask.at<cv::Vec3b>(k, m)[0]  > 0 || mask.at<cv::Vec3b>(k, m)[1] > 0 || mask.at<cv::Vec3b>(k, m)[2] > 0)
//				mergedMask.at<cv::Vec3b>(k, m) = White;
//		}
//		}
	threshold(maskTemp, binaryMask, 100, 255, cv::THRESH_BINARY);
//	namedWindow("Mask", CV_WINDOW_AUTOSIZE);
//	imshow("Mask", binaryMask);
	cvtColor(finalMask,finalMaskGray,CV_BGR2GRAY);
	threshold(finalMaskGray, binaryFinal, 100, 255, cv::THRESH_BINARY);
//	ROS_WARN_STREAM("Binary mask size = " << binaryMask.cols << "," << binaryMask.rows);

	bitwise_or(binaryMask,binaryFinal,binaryMerged);
	Mat_t mergedBin;
	cvtColor(binaryMerged,mergedBin,CV_GRAY2RGB);
	final = mergedBin;
	/// Show in a window
	cv::line(drawing, Point2d(0, HORIZON_TOP_), Point2d(drawing.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));
	cv::line(drawing, Point2d(0, HORIZON_BTM_), Point2d(drawing.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));
//	namedWindow("b", CV_WINDOW_AUTOSIZE);
//	imshow("b", finalMask);
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("After Fill Cont", mergedBin);


	cv::waitKey(3);

//		std::stringstream s;
//		s << "/home/srr/ObjectDetectionData/blob/0.png";
//		cv::imwrite(s.str(), drawing);
}

void BOOMStage::detectAnomalies(Mat_t& img, Mat_t& mask) {
	NODELET_INFO_STREAM("IN FILLHOLES IDENTIFY");
	Mat_t med, src_gray, normImg;
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	medianBlur(img, med, 11);

	normImg = med;
	cvtColor(normImg, src_gray, CV_BGR2GRAY);

	/// Detect edges using Threshold
	threshold(src_gray, threshold_output, thresh, 255, THRESH_BINARY);

	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_NONE, Point(0, 0));

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
			if((mask.at<cv::Vec3b>(center[i].y,center[i].x)[0] >0) && (radius[i] > 15 && radius[i] < 40)){
			drawContours(drawing, contours_poly, i, color, 1, 8,
					vector<Vec4i>(), 0, Point());
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2,
					8, 0);
			circle(drawing, center[i], (int) radius[i], color, 2, 8, 0);
			drawContours(src_gray, contours_poly, i, color, 1, 8,
					vector<Vec4i>(), 0, Point());
			rectangle(src_gray, boundRect[i].tl(), boundRect[i].br(), color, 2,
					8, 0);
			circle(src_gray, center[i], (int) radius[i], color, 2, 8, 0);
			cout << "Center of object[" << i << "]" << "= " << center[i].x
									<< "," << center[i].y << endl;
			DetectionPtr_t newDetection(new Detection_t());
			newDetection->first.first = center[i].x;
			newDetection->first.second = center[i].y;
			newDetection->second = Unknown;
			detection_list_.push_back(newDetection);
			}
		}
	}

	/// Show in a window
	cv::line(drawing, Point2d(0, HORIZON_TOP_), Point2d(drawing.cols, HORIZON_TOP_),
			Scalar(0, 255, 0));
	cv::line(drawing, Point2d(0, HORIZON_BTM_), Point2d(drawing.cols, HORIZON_BTM_),
			Scalar(0, 255, 0));

	imshow("Anomalies on norm", src_gray);
//
	cv::waitKey(3);
	imshow("Anomaly Contours", drawing);
//
	cv::waitKey(3);

//		std::stringstream s;
//		s << "/home/srr/ObjectDetectionData/blob/0.png";
//		cv::imwrite(s.str(), drawing);
}

void BOOMStage::computeDisparity()
{
	this->stereo_model.fromCameraInfo(this->left_info_, this->right_info_);
	Mat_t leftRect,rightRect;
	cvtColor(left_image_, leftRect, CV_BGR2GRAY);
	cvtColor(right_image_, rightRect, CV_BGR2GRAY);
	const cv::Mat_<uint8_t> img1_rect(leftRect.rows, leftRect.cols,
			const_cast<uint8_t*>(&leftRect.data[0]), rightRect.step);
	const cv::Mat_<uint8_t> img2_rect(rightRect.rows, rightRect.cols,
			const_cast<uint8_t*>(&rightRect.data[0]), rightRect.step);
	int heightL = img1_rect.rows;
	int widthL = img1_rect.cols;

	Mat_t disp(heightL, widthL, CV_16S);
	Mat_t dispn(heightL, widthL, CV_32F);


		int minDisp = 0;      //0         //-128-32;
		int numDisp = 192;       //80        //256+80;
		int SADSize = 21;				//10
		int P1 = 8 * SADSize * SADSize;
		int P2 = 32 * SADSize * SADSize;
		int disp12MaxDiff = -1; // 1;
		int preFilterCap = 31; //  2;
		int uniqueness = 15;
		int specSize = 100; //50 //20;   //reduces noise
		int specRange = 4;  //5 //1;

		cv::StereoBM stereoBM;
		cv::Ptr<CvStereoBMState> params = stereoBM.state;
		stereoBM.state->SADWindowSize = SADSize;
		stereoBM.state->preFilterCap = 31;
		stereoBM.state->minDisparity = minDisp;
		stereoBM.state->numberOfDisparities = numDisp;
		stereoBM.state->uniquenessRatio = uniqueness;
		stereoBM.state->textureThreshold = 10;
		stereoBM.state->speckleWindowSize = specSize;
		stereoBM.state->speckleRange = specRange;
		stereoBM.state->preFilterSize = 9;
		stereoBM(img1_rect, img2_rect, disp, CV_32F);

		geometry_msgs::PointStamped camera_point, world_point;
		for (int i = 0; i < (int) detection_list_.size(); i++) {
			Point2d obj_centroid(detection_list_.at(i)->first.first,
					detection_list_.at(i)->first.second);
			Point3d obj_3d;

			if (obj_centroid.x < disp.cols && obj_centroid.y < disp.rows) {

				float disp_val = disp.at<float>(obj_centroid.y, obj_centroid.x);
				this->stereo_model.projectDisparityTo3d(obj_centroid, disp_val,
						obj_3d);
				tf::Point detection(obj_3d.x, obj_3d.y, obj_3d.z);
				tf::pointTFToMsg(detection, camera_point.point);
				ros::Time tZero(0);
				camera_point.header.frame_id = "/upper_stereo_optical_frame";
				camera_point.header.stamp = tZero;
				world_point.header.frame_id = "/world";
				world_point.header.stamp = tZero;
				optimus_prime.waitForTransform("/world",
						camera_point.header.frame_id, ros::Time(0),
						ros::Duration(1.0));
				optimus_prime.transformPoint("/world", camera_point, world_point);
				tf::pointMsgToTF(camera_point.point, detection);
				watson_->addDetection(detection, detection_list_.at(i)->second);
			}
		}
		std::vector<tf::Point> detections;
		detection_list_.clear();
		watson_->shrink();
		geometry_msgs::PoseArrayPtr poses(new geometry_msgs::PoseArray);
		geometry_msgs::Pose tempPose;
		tempPose.orientation.w  = 1;
		if (watson_->getAllAboveConf(detections))
		{
			BOOST_FOREACH(std::vector<tf::Point>::value_type item, detections)
				{
					tf::pointTFToMsg(item,tempPose.position);
					poses->poses.push_back(tempPose);
				}
			this->pose_array_pub_.publish(poses);
		}

}

void BOOMStage::generateMsg() {

}
