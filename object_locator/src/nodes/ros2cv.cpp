#include <object_locator/ros2cv.h>


namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;



ImageConverter::ImageConverter()
: it_(nh_),
  WINDOWLeft ("Left Camera"),
  WINDOWRight ("Right Camera"),
  WINDOWDisparity ("Disparity"),
  gotLeft(false),
  gotRight(false)

{
	image_pub_ = it_.advertise("/out", 1);
	image_left_ = it_.subscribeCamera("/stereo_bottom/left/image_raw", 1, &ImageConverter::imageCbLeft, this);
	image_right_ = it_.subscribeCamera("/stereo_bottom/right/image_raw", 1, &ImageConverter::imageCbRight, this);
	disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);

	cv::namedWindow(WINDOWLeft);
	cv::namedWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(WINDOWLeft);
	cv::destroyWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
}

void ImageConverter::processImage(const sensor_msgs::Image& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW)
{
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


	cv::Mat img;
	img = cv_ptr->image;
	//    cv::imshow(WINDOW, img);
	cv::waitKey(3);

	image_pub_.publish(cv_ptr->toImageMsg());
}

void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	left_image = *msg;
	left_info  = *cam_info;
	gotLeft = true;
}
void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	right_image = *msg;
	right_info  = *cam_info;
	gotRight = true;
}
void ImageConverter::computeDisparity()
{
	processImage(left_image, mat_left, WINDOWLeft);
	processImage(right_image, mat_right, WINDOWRight);
	this->stereo_model.fromCameraInfo(this->left_info, this->right_info);
	//	  cv::StereoVar stereo;
	//	  stereo.maxDisp = 1.5;
	//	  stereo.minDisp = .3;
	//	  stereo(this->mat_left->image, this->mat_right->image, this->disparity);
	//	  cv::imshow(WINDOWDisparity, this->disparity);

	cv::Mat Kl(3,3,CV_64F, this->left_info.K.elems);
	cv::Mat Rl(3,3,CV_64F, this->left_info.R.elems);
	cv::Mat Pl(3,4,CV_64F, this->left_info.P.elems);
	cv::Mat Dl(1,5,CV_64F, this->left_info.D.data());
	uint heightL = this->left_info.height;
	uint widthL = this->left_info.width;

	cv::Mat Kr(3,3,CV_64F, this->right_info.K.elems);
	cv::Mat Rr(3,3,CV_64F, this->right_info.R.elems);
	cv::Mat Pr(3,4,CV_64F, this->right_info.P.elems);
	cv::Mat Dr(1,5,CV_64F, this->right_info.D.data());
	uint heightR = this->right_info.height;
	uint widthR = this->right_info.width;

	//	  CvMat Klm = Kl;
	//	  CvMat Rlm = Rl;
	//	  CvMat Plm = Pl;
	//	  CvMat Dlm = Dl;
	//
	//	  CvMat Krm = Kr;
	//	  CvMat Rrm = Rr;
	//	  CvMat Prm = Pr;
	//	  CvMat Drm = Dr;


	cv::Mat mx1(heightL, widthL, CV_32FC1);
	cv::Mat mx2(heightR, widthR, CV_32FC1);
	cv::Mat my1(heightL, widthL, CV_32FC1);
	cv::Mat my2(heightR, widthR, CV_32FC1);
	cv::Mat img1_rect(heightL, widthL, CV_8U);
	cv::Mat img2_rect(heightR, widthR, CV_8U);
	cv::Size size;
	size.height = heightL;
	size.width = widthL;


	cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, size, CV_32FC1, mx1, my1);
	cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, size, CV_32FC1,mx2, my2);



	cv::Mat left_gray;
	cv::Mat right_gray;

	cv::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	cv::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	cv::remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	cv::remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);


	cv::Mat disp(  heightL, widthL, CV_16S );
	cv::Mat vdisp( heightL, widthL, CV_8U );
	cv::Mat dispn( heightL, widthL, CV_32F );
	int minDisp = -300;
	int numDisp = 512+32;
	int SADSize = 17;
	int P1 = 8*SADSize*SADSize;
	int P2 = 32*SADSize*SADSize;
	int disp12MaxDiff = 1;
	int preFilterCap = 2;
	int uniqueness = 6;
	int specSize = 450;   //reduces noise
	int specRange = 10;
	cv::StereoSGBM stereoBM(minDisp, numDisp, SADSize, P1, P2, disp12MaxDiff, preFilterCap, uniqueness, specSize, specRange);

	stereoBM(img1_rect, img2_rect, disp);


	//    cv::erode(disp, disp, NULL, 2);
	//    cv::dilate(disp, disp, NULL, 2);

	//    cvConvertScale(disp, dispn, 1.0/16);
	//	 cv::filterSpeckles(disp, 200, 24, 13);
	//    cv::normalize( disp, vdisp, 0, 256, CV_MINMAX );
	cv::Mat vdisp1;

	cv::resize(disp, vdisp1,size);
	cv::imshow(WINDOWDisparity, vdisp1 );
	cv::imshow(WINDOWLeft, img1_rect);
	cv::imshow(WINDOWRight, img2_rect);
	cv::Mat point_cloud;
	this->stereo_model.projectDisparityImageTo3d(disp, point_cloud);
//	ROS_INFO_STREAM("Point Cloud Value: "<<point_cloud.at<unsigned int>(100,100));
	cv::waitKey(3);

}
void ImageConverter::computeDisparityCb(const ros::TimerEvent& event)
{
	if (gotLeft && gotRight)
	{
		computeDisparity();
		gotLeft = false;
		gotRight = false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
