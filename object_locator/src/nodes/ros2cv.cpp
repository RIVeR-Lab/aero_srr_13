#include <object_locator/ros2cv.h>


namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;
using namespace std;
using namespace cv;



ImageConverter::ImageConverter()
: it_(nh_),
  WINDOWLeft ("Left Camera"),
  WINDOWRight ("Right Camera"),
  WINDOWDisparity ("Disparity"),
  gotLeft(false),
  gotRight(false)

{
	image_pub_ = it_.advertise("/out", 1);
//	image_left_ = it_.subscribeCamera("prosilica/image_raw", 1, &ImageConverter::imageCbLeft, this);
	image_left_ = it_.subscribeCamera("/stereo_bottom/left/image_raw", 1, &ImageConverter::imageCbLeft, this);
	image_right_ = it_.subscribeCamera("/stereo_bottom/right/image_raw", 1, &ImageConverter::imageCbRight, this);
	image_left_ = it_.subscribeCamera("out", 1, &ImageConverter::imageCbLeft, this);

//	disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);
	cascade_path = "/home/srr/ObjectDetectionData/exec/cascadeHOGWHA/cascade.xml";
	ctr = 0;
	cv::namedWindow(WINDOWLeft);
//	cv::namedWindow(WINDOWRight);
//	cv::namedWindow(WINDOWDisparity);
	// new window
	HuethresH =0,
	HuethresL =0,
	SatthresL =0,
	SatthresH = 0,
	ValthresL =25,
	ValthresH = 100,
	erosionCount = 1,
	blurSize = 3;
//	cvNamedWindow("Color Tune",CV_WINDOW_NORMAL);
//	cvCreateTrackbar( "Hue UpperT","Color Tune", &HuethresH, 255, 0 );
//	cvCreateTrackbar ("Hue LowerT","Color Tune", &HuethresL,255, 0);
//	cvCreateTrackbar( "Sat UpperT","Color Tune", &SatthresH, 255, 0 );
//	cvCreateTrackbar( "Sat LowerT","Color Tune", &SatthresL, 255, 0 );
//	cvCreateTrackbar( "Val UpperT","Color Tune", &ValthresH, 255, 0 );
//	cvCreateTrackbar( "Val LowerT","Color Tune", &ValthresL, 255, 0 );
//	cvCreateTrackbar ("EroTime","Color Tune", &erosionCount,15, 0);
//	cvCreateTrackbar ("BlurSize","Color Tune", &blurSize,15, 0);

}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(WINDOWLeft);
//	cv::destroyWindow(WINDOWRight);
//	cv::destroyWindow(WINDOWDisparity);
	cvDestroyAllWindows();
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

//	std::stringstream s;
//	s << "/home/srr/ObjectDetectionData/samplesColor/" << ctr<<".png";
//	std::cout << s.str()<<std::endl;
	cv::Mat img;
	img = cv_ptr->image;
//	std::cout << "displaying image"<<std::endl;
//	    cv::imshow(WINDOW, img);


	  if( !cascade.load( cascade_path ) )
		  {
			  printf("--(!)Error loading\n");
		  }
//		    	 cv::GaussianBlur( img, img, cv::Size(9, 9), 2, 2 );

//	  imshow(WINDOW,img);

//		  	   int c = cv::waitKey(10);
//		  	         if( (char)c == 's' ) { cv::imwrite(s.str(), img); ctr++;}
//		    	 detectAndDisplay( img);
//	  	  	  	  test(img, WINDOW);
//		    	 tune(img,WINDOW);
		    	 image_pub_.publish(cv_ptr->toImageMsg());
}

void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	left_image = *msg;
	left_info  = *cam_info;
	gotLeft = true;
//	processImage(left_image, mat_left, WINDOWLeft);

}
void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	right_image = *msg;
	right_info  = *cam_info;
	gotRight = true;
}
void ImageConverter::tune(cv::Mat img, const char* WINDOW)
{
Mat img2;
Mat hue;			// hue channel
Mat hue1;			// Hue upper bound
Mat hue2;			// Hue lower bound
Mat hue3;			// hue color filtering
Mat sat;			// Sat channel
Mat sat1;			// Sat upper bound
Mat sat2;			// sat lower bound
Mat sat3;			// sat color filtering
Mat val;			// Val channel
Mat val1;			// Val upper bound
Mat val2;			// Val lower bound
Mat val3;			// Val color filtering
Mat erd;			// Erosion Image
Mat dia;			// dialate image
Mat HnS;			// sat and hue channel
Mat HSV;			// HSV color fiter detected
Mat cross = getStructuringElement(MORPH_CROSS, Size(5,5));
vector<Mat> slices;
// slide bar values




// make tune bar


cvtColor(img, img2,CV_BGR2HSV);
split(img2,slices);
slices[0].copyTo (hue); // get the hue channel
slices[1].copyTo(sat); // get the sat channel
slices[2].copyTo(val); // get the V channel

threshold (hue,hue1,HuethresL,255, CV_THRESH_BINARY); // get lower bound
threshold (hue, hue2,HuethresH,255, CV_THRESH_BINARY_INV); // get upper bound

hue3 = hue1 & hue2; // multiply 2 matrix to get the color range

// apply thresshold for Sat channel
threshold (sat,sat1,SatthresL,255, CV_THRESH_BINARY); // get lower bound
threshold (sat, sat2,SatthresH,255, CV_THRESH_BINARY_INV); // get upper bound
sat3 = sat1 & sat2; // multiply 2 matrix to get the color range

// apply thresshold for Val channel
threshold (val,val1,SatthresL,255, CV_THRESH_BINARY); // get lower bound
threshold (val, val2,SatthresH,255, CV_THRESH_BINARY_INV); // get upper bound
val3 = val1 & val2; // multiply 2 matrix to get the color range

HnS = sat3 & hue3;

erode(HnS,erd,cross,Point(-1,-1),erosionCount); // do erode
dilate(HnS,dia,cross,Point(-1,-1),erosionCount);// do dialate
// combine sat, val and hue filter together
HSV = sat3 & hue3 & val3;

// erode and dialation to reduce noise
erode(HSV,erd,cross,Point(-1,-1),erosionCount); // do erode
dilate(HSV,dia,cross,Point(-1,-1),erosionCount); // do dialate
imshow("HSV",HSV);
int c = cv::waitKey(10);
}
void ImageConverter::test(cv::Mat img, const char* WINDOW)
{
	cv::Mat img_gray, hsv,mask3, mask2,mask;
	std::vector<Vec3f> circles;
	Size k;
	Size k11;
	Size imgS;
	k.height = 21;
	k.width = 21;
	k11.height =11;
	k11.width = 11;
	Mat sel2;
	Mat sel1;
//	cv::cvtColor( img, img_gray, CV_BGR2GRAY );
	  cvtColor(img, hsv,CV_RGB2HSV);
	  inRange(hsv, Scalar(36,49, 25,0) , Scalar(66, 233, 100,0), mask );
//	  inRange(hsv, Scalar(170,50, 170,0) , Scalar(256, 180, 256,0), mask2 );
//	  bitwise_or(mask3,mask2,mask);
	  sel2 = getStructuringElement(MORPH_RECT, k, Point(10,10));
	  sel1 = getStructuringElement(MORPH_RECT, k11, Point(5,5));


	cv::GaussianBlur( mask, mask, cv::Size(9, 9), 2, 2 );

	  HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 2, mask.rows/4, 100, 40, 20, 200);
	  int i;
	  for( size_t i = 0; i < circles.size(); i++ )
	     {
	          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	          int radius = cvRound(circles[i][2]);

	          circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );    // draw the circle center
	          circle( img, center, radius, Scalar(255,0,0), 3, 8, 0 );// draw the circle outline
//	          circle(mask,center, 3, Scalar(0,255,0), -1, 8, 0 );
//	          circle(mask, center, radius, Scalar(0,0,255), 3, 8, 0 );
	     }
//	  imshow( WINDOW, hsv );
//	  imshow("img",img);
	  imshow("mask",mask);
	   cv::waitKey(3);
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

	//Auto grab info
//	cv::Mat Kl(3,3,CV_64F, this->left_info.K.elems);
//	cv::Mat Rl(3,3,CV_64F, this->left_info.R.elems);
//	cv::Mat Pl(3,4,CV_64F, this->left_info.P.elems);
//	cv::Mat Dl(1,5,CV_64F, this->left_info.D.data());
//	uint heightL = this->left_info.height;
//	uint widthL = this->left_info.width;
//
//	cv::Mat Kr(3,3,CV_64F, this->right_info.K.elems);
//	cv::Mat Rr(3,3,CV_64F, this->right_info.R.elems);
//	cv::Mat Pr(3,4,CV_64F, this->right_info.P.elems);
//	cv::Mat Dr(1,5,CV_64F, this->right_info.D.data());
//	uint heightR = this->right_info.height;
//	uint widthR = this->right_info.width;
// Manual Info
		cv::Mat Kl=(cv::Mat_<float>(3,3)<<1200.576576, 0, 660.281868, 0, 1201.521424, 363.013750, 0, 0, 1);
		cv::Mat Rl=(cv::Mat_<float>(3,3)<<0.999936,-.000728, 0.01281, 0.000728,1,0.000019,-0.011281,-0.000011,0.999936);
		cv::Mat Pl=(cv::Mat_<float>(3,4)<<1185.777062,0,641.420067,0,0,1185.777062,358.743057,0,0,0,1,0);
		cv::Mat Dl=(cv::Mat_<float>(1,5)<<-0.248229,0.107929,-0.000605,-0.000647,0);
		uint heightL = 734;
		uint widthL = 1292;


		cv::Mat Kr=(cv::Mat_<float>(3,3)<<1194.235157,0,644.186461,0,1195.051692, 355.258135, 0,0,1);
		cv::Mat Rr=(cv::Mat_<float>(3,3)<<0.999987,0.001926,0.004714,-0.001926,0.999998,-0.000020,-0.004714,0.000011,0.999989);
		cv::Mat Pr=(cv::Mat_<float>(3,4)<<1185.777062,0,641.420067,-116.860986,0,1185,777062,358.743057,0,0,0,1,0);
		cv::Mat Dr=(cv::Mat_<float>(1,5)<<-0.244669,0.097783,0.000396,0.000700,0);
		uint heightR = 734;
		uint widthR = 1292;


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
//   cascade.detectMultiScale( frame_gray, faces, 1.1, 35, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDA
//   cascade.detectMultiScale( frame_gray, faces, 1.1, 15, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDB
//   cascade.detectMultiScale( frame_gray, faces, 1.1, 35, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDD
   cascade.detectMultiScale( frame_gray, faces, 1.1, 295, 0, cv::Size(70, 100), cv::Size(150,215) ); // works for WHA

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
   cv::waitKey(3);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
