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
	this->left_camera = "left_camera";
	this->right_camera = "right_camera";
	this->disparity = "disparity";
	this->point_cloud = "point_cloud";
	this->optical_frame ="optical_frame";
	this->lower_bound_name = "lower_bound";
	this->upper_bound_name = "upper_bound";
	this->comparison_out_topic = "comparison_out_topic";
	this->lower_bound = 4;
	this->upper_bound = 100;
	this->output_topic = "output_topic";
	this->grass_level_name = "grass_level";
	this->getPrivateNodeHandle().getParam(this->left_camera,
			this->left_camera);
	this->getPrivateNodeHandle().getParam(this->right_camera,
			this->right_camera);
	this->getPrivateNodeHandle().getParam(this->disparity,
			this->disparity);
	this->getPrivateNodeHandle().getParam(this->point_cloud,
			this->point_cloud);
	this->getPrivateNodeHandle().getParam(this->optical_frame,
			this->optical_frame);
	this->getPrivateNodeHandle().getParam(this->output_topic,
			this->output_topic);
	this->getPrivateNodeHandle().getParam(this->lower_bound_name,
			this->lower_bound);
	this->getPrivateNodeHandle().getParam(this->upper_bound_name,
			this->upper_bound);
	this->getPrivateNodeHandle().getParam(this->HORIZON_TOP_NAME,
			this->HORIZON_TOP_);
	this->getPrivateNodeHandle().getParam(this->comparison_out_topic,
			this->comparison_out_topic);
	this->getPrivateNodeHandle().getParam(this->grass_level_name,
			this->grass_level);
	this->it_ = new image_transport::ImageTransport(this->getNodeHandle());
	this->HORIZON_TOP_NAME = "HORIZON";
	this->HORIZON_TOP_ = 125;
	this->HORIZON_BTM_ = 730;
	this->grass_level = .33;
	this->ZeroV[0] = 0;
	this->ZeroV[1] = 0;
	this->ZeroV[2] = 0;

	this->White[0] = 255;
	this->White[1] = 255;
	this->White[2] = 255;
	this->got_left_ = false;
	this->got_right_ = false;
//	this->left_header_ = 0;
//	this->left_encoding_ = enc::RGB8;

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
	this->train_ = true;
}

void BOOMStage::registerTopics() {
	this->image_left_ = it_->subscribeCamera(this->left_camera, 2,
			&BOOMStage::boomImageCbleft, this);
	this->image_right_ = it_->subscribeCamera(this->right_camera, 2,
				&BOOMStage::boomImageCbright, this);
//	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,2,&BOOMStage::boomImageCb,this);
	this->pose_array_pub_ = this->getNodeHandle().advertise<geometry_msgs::PoseArray>(this->output_topic,2);
	this->comparison_out_pub_ = this->getNodeHandle().advertise<geometry_msgs::PoseArray>(this->comparison_out_topic,2);
	this->disp_img_pub_ = this->getNodeHandle().advertise<sensor_msgs::Image>(this->disparity,2);
	this->point_cloud_pub_ = this->getNodeHandle().advertise<sensor_msgs::PointCloud2>(this->point_cloud,2);
}

void BOOMStage::boomImageCbleft(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& info) {
	cv_bridge::CvImagePtr img;
//	NODELET_INFO_STREAM("In Boom Image CB");
	try {
		img = cv_bridge::toCvCopy(msg, enc::RGB8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	left_image_ = img->image;
	left_info_ = *info;
	left_msg_ = *msg;
	got_left_ = true;
	computeDisparityCb();
	Mat_t src = img->image;
	Mat_t normImage,mask,finalMask,hsv2;
//	circleFind(*msg);
//	gmmRemove(msg,hsv2);
	grassRemove(*msg, normImage);
	maskCreate(*msg,mask);
//	fenceCheckerStd(normImage);
//	fillHoles(mask);
	blobIdentify(normImage,mask, finalMask);
	showAlpha(src,finalMask);
	detectAnomalies(normImage,finalMask);

}
void BOOMStage::boomImageCbright(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& info) {
	cv_bridge::CvImagePtr img;
//	NODELET_INFO_STREAM("In Boom Image CB");
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
inline bool isValidPoint(const cv::Vec3f& pt) {
	// Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
	// and zero disparities (point mapped to infinity).
	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z
			&& !std::isinf(pt[2]);
}
void BOOMStage::gmmRemove(const sensor_msgs::ImageConstPtr& msg, Mat_t& hsvImage)
{
	cv_bridge::CvImagePtr img;
//	Mat_t src = load_;
	try {
		img = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat_t source(img->image);
	Mat_t hsv;
	Mat_t hsv32F,hsv32Fnorm;
	cvtColor(source,hsv,CV_BGR2HSV);

	hsv.convertTo(hsv32F,CV_32FC3);
	normalize(hsv32F,hsv32Fnorm,0,1,CV_MINMAX);
	Vec3f HSV3 = hsv32Fnorm.at<cv::Vec3f>(100, 100);
	Vec3b RGB;
	Vec3f HSV;
	double Gratio, Rratio;
	ROS_INFO_STREAM("NORM OF 32HSV at 100,100 = " << HSV3[0]);
	Mat_t detected = Mat::zeros(source.size(), CV_8UC3);
	for(int k = 0; k< source.rows; k++)
	{
		for(int m = 0;m<source.cols; m++)
		{
			RGB = source.at<cv::Vec3b>(k, m);
			HSV = hsv32Fnorm.at<cv::Vec3f>(k,m);

			//ROS_INFO_STREAM("RGBd is  = " << (double)RGB[1]);

			Gratio = (double)RGB[1]/(double)(RGB[1]+RGB[2]+RGB[0]);
			Rratio = (double)RGB[2]/(double)(RGB[1]+RGB[2]+RGB[0]);

			//if((HSV[2] >0.95) && Gratio < 0.331 && std::abs(browness - 1) > 0.2)
			//remove high luminosity regions and regions of green and yellow
			if((HSV[2] >0.8) && Gratio < 0.331 && Rratio < 0.331)
				detected.at<cv::Vec3b>(k, m) = White;
			else
				detected.at<cv::Vec3b>(k, m) = ZeroV;

		}
	}
	hsvImage = detected;

//	imshow("b4detection", detected);
//
//	//dilation (morphological operation to take away salt + pepper noise and increase size of detected blobs)
//	int elem_type = MORPH_CROSS;
//	int elem_size = 1;
//
//	Mat element = getStructuringElement( elem_type, Size( 2*elem_size + 1, 2*elem_size+1 ), Point( elem_size, elem_size ) );
//	morphologyEx(detected, detected, MORPH_OPEN, element);// do morphological opening to remove small specks and enlarge larger detections


	imshow("detection", detected);
	waitKey(3);

}


void BOOMStage::grassRemove(const sensor_msgs::Image& msg, Mat_t& normImage) {
//	NODELET_INFO_STREAM("IN BLOB GRASS REMOVE");
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
			if ((nG > grass_level)
					|| ((std::abs(browness - 1) < .2) && (whiteness < .9))) {
				norma.at<cv::Vec3b>(x, y) = ZeroV;

			}
			else {
				norma.at<cv::Vec3b>(x, y) = White;

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


//
imshow("norm", norma);
waitKey(3);


}
void BOOMStage::maskCreate(const sensor_msgs::Image& msg, Mat_t& maskt) {
//	NODELET_INFO_STREAM("IN MASK CREATE");
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
			if ((nG > grass_level)
					|| ((std::abs(browness - 1) < .2) && (whiteness < .9))) {
				mask.at<cv::Vec3b>(x, y) = White;
			}
			else {
				mask.at<cv::Vec3b>(x, y) = ZeroV;
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
//	NODELET_INFO_STREAM("IN BLOB IDENTIFY");
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
//	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
//	imshow("After Fill Cont", mergedBin);


	cv::waitKey(3);

//		std::stringstream s;
//		s << "/home/srr/ObjectDetectionData/blob/0.png";
//		cv::imwrite(s.str(), drawing);
}

void BOOMStage::detectAnomalies(Mat_t& img, Mat_t& mask) {
//	NODELET_INFO_STREAM("IN FILLHOLES IDENTIFY");
	Mat_t med, src_gray, normImg;
	int thresh = 0;
	int max_thresh = 255;
	RNG rng(12345);
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
//	medianBlur(img, med, 11);

	normImg = img;
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
			if((mask.at<cv::Vec3b>(center[i].y,center[i].x)[0] >0) && (radius[i] > this->lower_bound && radius[i] < this->upper_bound)){
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
			rectangle(left_image_, boundRect[i].tl(), boundRect[i].br(), color, 2,
					8, 0);
			circle(left_image_, center[i], (int) radius[i], color, 2, 8, 0);
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
	imshow("Anomaly Contours", left_image_);
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
		sensor_msgs::ImagePtr disp_msg = boost::make_shared<
				sensor_msgs::Image>();
		cv_bridge::CvImage carrier;
		disp_msg->header = left_msg_.header;
		disp_msg->encoding = enc::TYPE_32FC1;
		carrier.image = disp;
		carrier.toImageMsg(*disp_msg);
		this->disp_img_pub_.publish(disp_msg);
		cv::Mat_<cv::Vec3f> points_mat_;
			this->stereo_model.projectDisparityImageTo3d(disp, points_mat_, true);
			cv::Mat_<cv::Vec3f> mat = points_mat_;

			sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<
					sensor_msgs::PointCloud2>();
			points_msg->header = left_msg_.header;
			points_msg->height = mat.rows;
			points_msg->width = mat.cols;
			points_msg->fields.resize(4);
			points_msg->fields[0].name = "x";
			points_msg->fields[0].offset = 0;
			points_msg->fields[0].count = 1;
			points_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
			points_msg->fields[1].name = "y";
			points_msg->fields[1].offset = 4;
			points_msg->fields[1].count = 1;
			points_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
			points_msg->fields[2].name = "z";
			points_msg->fields[2].offset = 8;
			points_msg->fields[2].count = 1;
			points_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
			points_msg->fields[3].name = "rgb";
			points_msg->fields[3].offset = 12;
			points_msg->fields[3].count = 1;
			points_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;

			//points_msg->is_bigendian = false; ???
			static const int STEP = 16;
			points_msg->point_step = STEP;
			points_msg->row_step = points_msg->point_step * points_msg->width;
			points_msg->data.resize(points_msg->row_step * points_msg->height);
			points_msg->is_dense = false; // there may be invalid points

			float bad_point = std::numeric_limits<float>::quiet_NaN();
			int offset = 0;
			for (int v = 0; v < mat.rows; ++v) {
				for (int u = 0; u < mat.cols; ++u, offset += STEP) {
					if (isValidPoint(mat(v, u))) {
						// x,y,z,rgba
						memcpy(&points_msg->data[offset + 0], &mat(v, u)[0],
								sizeof(float));
						memcpy(&points_msg->data[offset + 4], &mat(v, u)[1],
								sizeof(float));
						memcpy(&points_msg->data[offset + 8], &mat(v, u)[2],
								sizeof(float));
					} else {
						memcpy(&points_msg->data[offset + 0], &bad_point,
								sizeof(float));
						memcpy(&points_msg->data[offset + 4], &bad_point,
								sizeof(float));
						memcpy(&points_msg->data[offset + 8], &bad_point,
								sizeof(float));
					}
				}
			}
			// Fill in color
			namespace enc = sensor_msgs::image_encodings;
			const std::string& encoding = left_msg_.encoding;
			offset = 0;
			if (encoding == enc::MONO8) {
				const cv::Mat_<uint8_t> color(right_image_.rows, right_image_.cols,
						(uint8_t*) &right_image_.data[0], right_image_.step);
				for (int v = 0; v < mat.rows; ++v) {
					for (int u = 0; u < mat.cols; ++u, offset += STEP) {
						if (isValidPoint(mat(v, u))) {
							uint8_t g = color(v, u);
							int32_t rgb = (g << 16) | (g << 8) | g;
							memcpy(&points_msg->data[offset + 12], &rgb,
									sizeof(int32_t));
						} else {
							memcpy(&points_msg->data[offset + 12], &bad_point,
									sizeof(float));
						}
					}
				}
			} else if (encoding == enc::RGB8) {
				const cv::Mat_<cv::Vec3b> color(right_image_.rows, right_image_.cols,
						(cv::Vec3b*) &right_image_.data[0], right_image_.step);
				for (int v = 0; v < mat.rows; ++v) {
					for (int u = 0; u < mat.cols; ++u, offset += STEP) {
						if (isValidPoint(mat(v, u))) {
							const cv::Vec3b& rgb = color(v, u);
							int32_t rgb_packed = (rgb[0] << 16) | (rgb[1] << 8)
									| rgb[2];
							memcpy(&points_msg->data[offset + 12], &rgb_packed,
									sizeof(int32_t));
						} else {
							memcpy(&points_msg->data[offset + 12], &bad_point,
									sizeof(float));
						}
					}
				}
			} else if (encoding == enc::BGR8) {
				const cv::Mat_<cv::Vec3b> color(right_image_.rows, right_image_.cols,
						(cv::Vec3b*) &right_image_.data[0], right_image_.step);
				for (int v = 0; v < mat.rows; ++v) {
					for (int u = 0; u < mat.cols; ++u, offset += STEP) {
						if (isValidPoint(mat(v, u))) {
							const cv::Vec3b& bgr = color(v, u);
							int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8)
									| bgr[0];
							memcpy(&points_msg->data[offset + 12], &rgb_packed,
									sizeof(int32_t));
						} else {
							memcpy(&points_msg->data[offset + 12], &bad_point,
									sizeof(float));
						}
					}
				}
			} else {
				ROS_WARN_THROTTLE(30,
						"Could not fill color channel of the point cloud, "
								"unsupported encoding '%s'", encoding.c_str());
			}

			this->point_cloud_pub_.publish(points_msg);
			//*********Oct tree stuff *************//
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tcloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*points_msg,*cloud);
			pcl_ros::transformPointCloud("/world", *cloud, *tcloud,optimus_prime);

			float resolution = 2.5f;
			pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

		    octree.setInputCloud (tcloud);
		    octree.addPointsFromInputCloud ();

		    pcl::PointXYZ searchPoint;


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
				searchPoint.x = detection.getX();
						searchPoint.y = detection.getY();
						searchPoint.z = detection.getZ();

						int K = 10;
							  std::vector<int> pointIdxVec;
						  std::vector<int> pointIdxNKNSearch;
						  std::vector<float> pointNKNSquaredDistance;

//						  std::cout << "K nearest neighbor search at (" << searchPoint.x
//						            << " " << searchPoint.y
//						            << " " << searchPoint.z
//						            << ") with K=" << K << std::endl;

						  if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						  {
							  float sumx =0.0;
							  float sumy =0.0;
							  float sumz =0.0;
						    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
						    {
//						      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
//						                << " " << cloud->points[ pointIdxNKNSearch[i] ].y
//						                << " " << cloud->points[ pointIdxNKNSearch[i] ].z
//						                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
						    	sumx = tcloud->points[ pointIdxNKNSearch[i] ].x + sumx;
						    	sumy = tcloud->points[ pointIdxNKNSearch[i] ].y + sumy;
						    	sumz = tcloud->points[ pointIdxNKNSearch[i] ].z + sumz;
						    }
						    xAvgVal_ = sumx/pointIdxNKNSearch.size ();
						    yAvgVal_ = sumy/pointIdxNKNSearch.size ();
						    kAvgVal_ = sumz/pointIdxNKNSearch.size ();

						  }
						  ROS_WARN_STREAM("Average value at point in cloud = " << kAvgVal_);
					  	  detection.setX(xAvgVal_);
					  	  detection.setY(yAvgVal_);
						  detection.setZ(kAvgVal_);
				tf::pointTFToMsg(detection, camera_point.point);
				ros::Time tZero(0);
				camera_point.header.frame_id = this->optical_frame;
				camera_point.header.stamp = tZero;
				world_point.header.frame_id = "/world";
				world_point.header.stamp = tZero;
				optimus_prime.waitForTransform("/world",
						camera_point.header.frame_id, ros::Time(0),
						ros::Duration(1.0));
				optimus_prime.transformPoint("/world", camera_point, world_point);
				tf::pointMsgToTF(world_point.point, detection);
				watson_->addDetection(detection, detection_list_.at(i)->second);
			}
		}
		std::vector<tf::Point> detections;
		detection_list_.clear();
		watson_->shrink();
		geometry_msgs::PoseArrayPtr poses(new geometry_msgs::PoseArray);
		poses->header.frame_id = "/world";
		poses->header.stamp = left_msg_.header.stamp;
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
			this->comparison_out_pub_.publish(poses);
			ROS_ERROR_STREAM("Sent color msg from color Detector");
		}
		ROS_WARN_STREAM("Number of detections in list = " << detections.size());
		Mat_t cmapped;
		disp.convertTo(cmapped, CV_8U);
		imshow("disparty",cmapped);
		waitKey(3);

}


void BOOMStage::generateMsg() {

}
