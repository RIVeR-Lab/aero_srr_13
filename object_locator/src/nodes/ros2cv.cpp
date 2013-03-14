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
	image_left_ = it_.subscribeCamera("prosilica/image_raw", 1, &ImageConverter::imageCbLeft, this);
//	image_right_ = it_.subscribeCamera("/stereo_bottom/right/image_raw", 1, &ImageConverter::imageCbRight, this);
//	disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);
	cascade_path = "/home/srr/ObjectDetectionData/exec/cascadeHOGBlur/cascade.xml";
	ctr = 0;
	cv::namedWindow(WINDOWLeft);
	cv::namedWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(WINDOWLeft);
	cv::destroyWindow(WINDOWRight);
	cv::destroyWindow(WINDOWDisparity);
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

	std::stringstream s;
	s << "/home/srr/ObjectDetectionData/samplesCam3/" << ctr<<".png";
	std::cout << s.str()<<std::endl;
	cv::Mat img;
	img = cv_ptr->image;
	//    cv::imshow(WINDOW, img);
	   int c = cv::waitKey(10);
	         if( (char)c == 's' ) { cv::imwrite(s.str(), img); ctr++;}
	image_pub_.publish(cv_ptr->toImageMsg());
	  if( !cascade.load( cascade_path ) )
		  {
			  printf("--(!)Error loading\n");
		  }
//		    	 cv::GaussianBlur( img, img, cv::Size(9, 9), 2, 2 );
//		    	 detectAndDisplay( img);
}

void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	left_image = *msg;
	left_info  = *cam_info;
	gotLeft = true;
	processImage(left_image, mat_left, WINDOWLeft);

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
void ImageConverter::detectAndDisplay( cv::Mat frame )
{

//	  std::cout << "running" << std::endl;
   std::vector<cv::Rect> faces;
   cv::Mat frame_gray;

   cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
   cv::equalizeHist( frame_gray, frame_gray );

   //-- Detect faces
   cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0, cv::Size(80, 80) );

   for( size_t i = 0; i < faces.size(); i++ )
    {
      cv::Mat faceROI = frame_gray( faces[i] );

      //-- In each face, detect eyes

         //-- Draw the face
         cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
         cv::ellipse( frame, center, cv::Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 0 ), 2, 8, 0 );
//   	  std::cout << "stuck" << std::endl;

    }
   //-- Show what you got
   cv::imshow( WINDOWLeft, frame );

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
