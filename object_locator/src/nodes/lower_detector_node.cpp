#include <object_locator/lower_detector_node.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include "ObjectLocatorParams.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <image_transport/subscriber_filter.h>
//#include <opencv2/gpu/stream_accessor.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;
using namespace std;
using namespace cv;



ImageConverter::ImageConverter()
: it_(nh_),
  WINDOWLeft ("Left Camera"),
  WINDOWRight ("Right Camera"),
  WINDOWDisparity ("Disparity"),
  sherlock(.5, .15,.05, .5),
  gotLeft(false),
  gotRight(false)


{
	//**********enable CUDA*****************
	CUDA_ENABLED = 0;


	//********ROS subscriptions and published topics***************
	ObjLocationPub = nh_.advertise<aero_srr_msgs::ObjectLocationMsg>("ObjectPose",2);
	image_pub_ = it_.advertise("/out", 1);
//	image_left_  = it_.subscribeCamera("/stereo/left/image_rect_color", 1, &ImageConverter::imageCbLeft, this);
//	image_right_ = it_.subscribeCamera("/stereo/right/image_rect_color", 1, &ImageConverter::imageCbRight, this);
//	disp_image_sub_ = nh_.subscribe("/stereo_camera/disparity",1, &ImageConverter::imageCbRight, this);
	left_rect_sub_ = nh_.subscribe("/stereo_camera/left/image_rect_color",1, &ImageConverter::rectLeftCb, this);
	right_rect_sub_ = nh_.subscribe("/stereo_camera/right/image_rect_color",1, &ImageConverter::rectRightCb, this);

	//	image_left_ = it_.subscribeCamera("prosilica/image_raw", 1, &ImageConverter::imageCbLeft, this);
	//	image_left_ = it_.subscribeCamera("out", 1, &ImageConverter::imageCbLeft, this);

	//********ROS Timer for Disparity image cb**************
	disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);

	//Cascade Trained xml file locations
	cascade_path_WHA = "/home/srr/ObjectDetectionData/exec/cascadeWHAground/cascade.xml";
	cascade_path_PINK = "/home/srr/ObjectDetectionData/exec/cascadePINKBALL/cascade.xml";
	cascade_path_WHASUN = "/home/srr/ObjectDetectionData/exec/cascadeWHAOutside/cascade.xml";
	ctrLeft = 0;
	ctrRight = 0;
	cv::namedWindow(WINDOWLeft);
	cv::namedWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
	objset = false;



}

ImageConverter::~ImageConverter()
{
	cv::destroyWindow(WINDOWLeft);
	cv::destroyWindow(WINDOWRight);
	cv::destroyWindow(WINDOWDisparity);
	//	cvDestroyAllWindows();
}

void ImageConverter::processImage(const sensor_msgs::Image& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW)
{
	ROS_INFO_STREAM("encoding = " << msg.encoding);
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
//

//	Mat_t img(cv_ptr->image);
	//	std::cout << "displaying image"<<std::endl;
	//	    cv::imshow(WINDOW, img);




//		  imshow(WINDOW,img);


	//		    	 detectAndDisplay( img);
	//	  	  	  	  test(img, WINDOW);
	//		    	 tune(img,WINDOW);
//	image_pub_.publish(cv_ptr->toImageMsg());
}
void ImageConverter::saveImage(const sensor_msgs::Image& msg, cv_bridge::CvImagePtr& cv_ptr, int O)
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

	Mat_t img(cv_ptr->image);
	std::stringstream s,d;
	s << "/home/srr/ObjectDetectionData/Stereo/Left/" << ctrLeft<<".png";
//	d << "/home/srr/ObjectDetectionData/Stereo/Right/" << ctrRight<<".png";
	std::cout << s.str()<<std::endl;
	std::cout << d.str()<<std::endl;
	   int c = cv::waitKey(5);
	   if(O == 0){if( (char)c == 's' ) { cv::imwrite(s.str(), img); ctrLeft++;}}
	   else{if( (char)c == 'd' ) { cv::imwrite(d.str(), img); ctrRight++;}}
}
void ImageConverter::rectLeftCb(const sensor_msgs::ImageConstPtr& msg)
{
	left_image = *msg;
	gotLeft = true;
}

void ImageConverter::rectRightCb(const sensor_msgs::ImageConstPtr& msg)
{
	right_image = *msg;
	gotRight = true;
}
void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	left_image = *msg;
	left_info  = *cam_info;
	gotLeft = true;
//	detectAndDisplay(left_image,mat_left,WINDOWLeft);
//		saveImage(left_image, mat_left,0);

}
void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{

	right_image = *msg;
	right_info  = *cam_info;
	gotRight = true;
//	saveImage(right_image, mat_right,1);
}


void ImageConverter::computeDisparity()
{
//	processImage(left_image, mat_left, WINDOWLeft);
//	processImage(right_image, mat_right, WINDOWRight);
//	ROS_INFO_STREAM("Finished acquiring images");
	Mat_t leftRect, img1_rect;
	Mat_t rightRect,img2_rect;
//	leftRect  =  imread("/home/srr/ObjectDetectionData/Tskuba/ALeft.jpg", CV_LOAD_IMAGE_COLOR);
//	rightRect  = imread("/home/srr/ObjectDetectionData/Tskuba/ARight.jpg", CV_LOAD_IMAGE_COLOR);
//	Mat_t img1_rect(leftRect.rows,leftRect.cols,CV_8U);
//	Mat_t img2_rect(leftRect.rows,leftRect.cols,CV_8U);
	cvtColor(mat_left->image,img1_rect, CV_BGR2GRAY);
	cvtColor(mat_right->image,img2_rect, CV_BGR2GRAY);
//	this->stereo_model.updateQ();
/**this->stereo_model.fromCameraInfo(this->left_info, this->right_info);
	//	  cv::StereoVar stereo;
	//	  stereo.maxDisp = 1.5;
	//	  stereo.minDisp = .3;
	//	  stereo(this->mat_left->image, this->mat_right->image, this->disparity);
	//	  cv::imshow(WINDOWDisparity, this->disparity);

	//Auto grab info
	Mat_t Kl(3,3,CV_64F, this->left_info.K.elems);
	Mat_t Rl(3,3,CV_64F, this->left_info.R.elems);
	Mat_t Pl(3,4,CV_64F, this->left_info.P.elems);
	Mat_t Dl(1,5,CV_64F, this->left_info.D.data());
	uint heightL = this->left_info.height;
	uint widthL = this->left_info.width;
	//	uint heightL = 367;
	//	uint widthL = 646;

	Mat_t Kr(3,3,CV_64F, this->right_info.K.elems);
	Mat_t Rr(3,3,CV_64F, this->right_info.R.elems);
	Mat_t Pr(3,4,CV_64F, this->right_info.P.elems);
	Mat_t Dr(1,5,CV_64F, this->right_info.D.data());
	uint heightR = this->right_info.height;
	uint widthR = this->right_info.width;
	//	uint heightR = 367;
	//		uint widthR = 646;


	Mat_t mx1(heightL, widthL, CV_32FC1);
	Mat_t mx2(heightR, widthR, CV_32FC1);
	Mat_t my1(heightL, widthL, CV_32FC1);
	Mat_t my2(heightR, widthR, CV_32FC1);
	Mat_t img1_rect(heightL, widthL, CV_8U);
	Mat_t img2_rect(heightR, widthR, CV_8U);
	cv::Size size;
	size.height = heightL;
	size.width = widthL;


	cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, size, CV_32FC1, mx1, my1);
	cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, size, CV_32FC1,mx2, my2);



	Mat_t left_gray;
	Mat_t right_gray;
	Mat_t Lds_img;
	Mat_t Rds_img;
	//	resize(mat_left->image,Lds_img,size);
	//	resize(mat_right->image,Rds_img,size);
**/
#ifdef CUDA_ENABLED
	gpu::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	gpu::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	gpu::remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	gpu::remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

#else
	/**
	cv::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	cv::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
 **/
#endif
	int heightL = img1_rect.rows;
	int widthL = img1_rect.cols;
	Mat_t disp(  heightL, widthL, CV_16S );
//	Mat_t vdisp( heightL, widthL, CV_32FC1 );
	Mat_t dispn( heightL, widthL, CV_32F );

	int minDisp = 0;      //0         //-128-32;
	int numDisp = 96;       //80        //256+80;
	int SADSize = 9;				//10
	int P1 =  8*SADSize*SADSize;
	int P2 = 32*SADSize*SADSize;
	int disp12MaxDiff =  -1	; // 1;
	int preFilterCap =   31; //  2;
	int uniqueness = 5;
	int specSize =   100; //50 //20;   //reduces noise
	int specRange = 20  ;  //5 //1;

#ifdef CUDA_ENABLED

	gpu::StereoBM_GPU gpuBM((StereoBM::BASIC_PRESET,numDisp, SADSize));
	gpuBM(img1_rect, img2_rect, disp);

#else
//	cv::StereoSGBM stereoSGBM(minDisp,numDisp, SADSize);
	cv::StereoSGBM stereoSGBM(minDisp, numDisp, SADSize, P1, P2, disp12MaxDiff, preFilterCap, uniqueness, specSize, specRange, false);
	stereoSGBM(img1_rect, img2_rect, disp);

//	cv::StereoBM stereoBM(StereoBM::BASIC_PRESET,numDisp, SADSize);
//		stereoBM(img1_rect, img2_rect, disp);
	std::stringstream s,d;
		s << "/home/srr/ObjectDetectionData/Disparity1.png";
		cv::imwrite(s.str(), dispn);
	    disp.convertTo(dispn, -1,1.0/16);
		Mat_t cmapped;
		dispn.convertTo(cmapped,CV_8U);
		cv::imshow(WINDOWDisparity, cmapped );
		cv::waitKey(3);
#endif
	//    cv::erode(disp, disp, NULL, 2);
	//    cv::dilate(disp, disp, NULL, 2);


	//	 cv::filterSpeckles(disp, 200, 24, 13);
/**	normalize( disp, vdisp, 0, 256, CV_MINMAX );
**/
//    Mat_t vdisp1(  heightL, widthL, CV_32FC1 );;

#ifdef CUDA_ENABLED
	gpu::resize(disp, vdisp1,size);
#else
//	cv::resize(disp, vdisp1,size);
#endif

	//	cv::imshow(WINDOWLeft, img1_rect);
	//	cv::imshow(WINDOWRight, img2_rect);
	Mat_t point_cloud;


	cv::Point3d real_xyz;
	geometry_msgs::PointStamped camera_point, world_point;
	for(int i = 0; i< (int)detection_list_.size(); i++)
	{
//		cout << endl;
//		cout << "In detection #"<< i+1 << "/"<< detection_list_WHA.size() <<endl;
		Point2d obj_centroid(detection_list_.at(i)->first.first,detection_list_.at(i)->first.second);
		Point3d obj_3d;


//		std::cout << "Checking disparity at  " << obj_centroid.x <<","<< obj_centroid.y << std::endl;
//		std::cout << "Range (rows,cols): " << vdisp1.rows <<","<< vdisp1.cols << std::endl;
		if(obj_centroid.x < disp.cols && obj_centroid.y < disp.rows)
		{
//			cout << "Getting Disparity" <<endl;
			float disp_val = dispn.at<uchar>(obj_centroid.y,obj_centroid.x);
//			cout << "Recieved Disparity of "<< disp_val <<endl;
			//			cv::ellipse( vdisp1, obj_centroid, cv::Size( 50, 114), 0, 0, 360, 0, 2, 8, 0 );
			this->stereo_model.projectDisparityTo3d(obj_centroid,disp_val,obj_3d);
//			cout << "Disp: "<< disp_val << endl << "X: "<< obj_3d.x << endl << "Y: " << obj_3d.y << endl << "Z: " << obj_3d.z << endl;
			tf::Point detection(obj_3d.x,obj_3d.y, obj_3d.z);
//			cout << "adding detection to camera_point" <<endl;
			tf::pointTFToMsg(detection, camera_point.point);
			ros::Time tZero(0);
			camera_point.header.frame_id = "/stereo_bottom/center";
			camera_point.header.stamp = tZero;
			world_point.header.frame_id = "/world";
			world_point.header.stamp = tZero;
//			cout << "Transforming camera to world" <<endl;
			optimus_prime.waitForTransform("/world", camera_point.header.frame_id, ros::Time(0), ros::Duration(1.0));
			optimus_prime.transformPoint("/world",camera_point, world_point);
//			cout << "Adding TFT to msg" <<endl;
			tf::pointMsgToTF(world_point.point,detection);
			sherlock.addDetection(detection, detection_list_.at(i)->second);
//			cout << "Added detection to manager" <<endl;
		}

	}
	tf::Point detection;
//	cout << "Clearing list" <<endl;
	detection_list_.clear();
//	cout << "Shrinking Det/ection manager list" <<endl;
	sherlock.shrink();
//	cout << "Finished shrinking list" <<endl;
	double confidence;
	object_type type;
	if(sherlock.getDetection(detection, type, confidence))
	{
		std::string typeString;
		switch(type)
		{
		case WHA:
			typeString = "White Hook Object";
			break;
		case PINK_BALL:
			typeString = "Pink Tennis Ball";
			break;
		default:
			typeString = "Unknown";
			break;

		}
		cout<<"I Got A Detection: "<< endl << "X:" << detection.getX()
								           <<", Y: "<< detection.getY()
								           << ", Z: " << detection.getZ()
								           <<", "<< confidence<<", of type: "<< typeString << std::endl;
		aero_srr_msgs::ObjectLocationMsg msg;

		msg.header.frame_id = camera_point.header.frame_id;
		msg.header.stamp = ros::Time::now();
		msg.pose.header.frame_id = camera_point.header.frame_id;
		msg.pose.header.stamp = ros::Time::now();
		buildMsg(detection, msg.pose);
		ObjLocationPub.publish(msg);
	}

//	cmapped = gray2bgr(vdisp1);

	cv::imshow(WINDOWLeft, img1_rect );
	cv::waitKey(3);
	cv::imshow(WINDOWRight, img2_rect );
		cv::waitKey(3);


}
Mat_t ImageConverter::gray2bgr(Mat_t img)
{
	Mat_t BGR( img.rows, img.cols, CV_32FC3 );
	for (int i = 0; i <img.rows; i ++)
	{
		for(int j=0; j<img.cols;j++)
		{
			BGR.at<Vec3i>(i,j)[0] =  std::abs(sin((img.at<double>(i,j))*2.0*3.14+0.0*3.14));
			BGR.at<Vec3i>(i,j)[1] = std::abs(sin((img.at<double>(i,j))*2.0*3.14+(-0.1)*3.14));
			BGR.at<Vec3i>(i,j)[2]= std::abs(sin((img.at<double>(i,j))*2.0*3.14+(-0.3)*3.14));
			cout <<"D:" <<img.at<uchar>(i,j) << endl;
			cout <<"S:" <<img.at<Scalar>(i,j) << endl;
		}
	}


	return BGR;
}

void ImageConverter::buildMsg(const tf::Point& point, geometry_msgs::PoseStamped& msg) const
{
	tf::pointTFToMsg(point,msg.pose.position);
	tf::Quaternion q;
	q.setRPY(0,0,0);
	q.normalize();
	tf::quaternionTFToMsg(q,msg.pose.orientation);
}



void ImageConverter::computeDisparityCb(const ros::TimerEvent& event)
{
	if (gotLeft && gotRight && (left_image.header.stamp == right_image.header.stamp))
	{
		computeDisparity();
		gotLeft = false;
		gotRight = false;
	}
//	computeDisparity();
}
void ImageConverter::detectAndDisplay( const sensor_msgs::Image& msg, cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW)
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
	Mat_t frame;
	frame = cv_ptr->image;
	if( !cascade_WHA.load( cascade_path_WHA))
	{
		printf("--(!)Error loading\n");
	}

	if( !cascade_PINK.load(cascade_path_PINK))
		{
			printf("--(!)Error loading\n");
		}
	if( !cascade_WHASUN.load(cascade_path_WHASUN))
		{
			printf("--(!)Error loading\n");
		}
	cv::GaussianBlur( frame, frame, cv::Size(9, 9), 2, 2 );

	std::vector<cv::Rect> WHA_faces, PINK_faces, SUN_faces;
	std::vector<std::vector<cv::Rect> > Detections;

	Mat_t frame_gray;

	cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	cv::equalizeHist( frame_gray, frame_gray );

	//-- Detect faces
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 35, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDA
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 15, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDB
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 30, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDD 35
//	cascade.detectMultiScale( frame_gray, faces, 1.1, 455, 0, cv::Size(70, 100), cv::Size(150,215) ); // works for WHA
//	cascade.detectMultiScale( frame_gray, faces, 1.1, 55, 0, cv::Size(75, 112), cv::Size(120, 190) ); // works for WHA comb
//	cascade.detectMultiScale( frame_gray, faces, 1.1, 25, 0, cv::Size(70, 100), cv::Size(150, 215) ); // works for WHA 7 samp close
//	cascade.detectMultiScale( frame_gray, faces, 1.1, 30, 0, cv::Size(40, 70), cv::Size(70, 100) ); // works for WHA 007
	cascade_WHA.detectMultiScale( frame_gray, WHA_faces, 1.1, 5, 0, cv::Size(52,59), cv::Size(85, 90) ); // works for WHAground !&
	cascade_PINK.detectMultiScale( frame_gray, PINK_faces, 1.1, 20, 0, cv::Size(45, 45), cv::Size(80, 80) ); // works for PINK !&
	cascade_WHASUN.detectMultiScale( frame_gray, SUN_faces, 1.1, 20, 0, cv::Size(45, 45), cv::Size(80, 80) ); // works for PINK !&


	for( size_t i = 0; i < WHA_faces.size(); i++ )
	{
//		cout << "Entered circle drawing loop" << endl;


		Mat_t faceROI = frame_gray( WHA_faces[i] );

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center( WHA_faces[i].x + WHA_faces[i].width/2, WHA_faces[i].y + WHA_faces[i].height/2 );

		cv::ellipse( frame, center, cv::Size( WHA_faces[i].width/2, WHA_faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 0 ), 2, 8, 0 );

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first  = center.x;
		newDetection->first.second = center.y;
		newDetection->second       = WHA;
		detection_list_.push_back(newDetection);



	}
	for( size_t j = 0; j < PINK_faces.size(); j++ )
	{
//		cout << "Entered circle drawing loop" << endl;


		Mat_t faceROI = frame_gray( PINK_faces[j] );

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center( PINK_faces[j].x + PINK_faces[j].width/2, PINK_faces[j].y + PINK_faces[j].height/2 );
		cv::ellipse( frame, center, cv::Size( PINK_faces[j].width/2, PINK_faces[j].height/2), 0, 0, 360, cv::Scalar( 0, 255, 0 ), 2, 8, 0 );

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first  = center.x;
		newDetection->first.second = center.y;
		newDetection->second       = PINK_BALL;
		detection_list_.push_back(newDetection);



	}
	for( size_t j = 0; j < SUN_faces.size(); j++ )
	{
//		cout << "Entered circle drawing loop" << endl;


		Mat_t faceROI = frame_gray( SUN_faces[j] );

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center( SUN_faces[j].x + SUN_faces[j].width/2, SUN_faces[j].y + SUN_faces[j].height/2 );
		cv::ellipse( frame, center, cv::Size( SUN_faces[j].width/2, SUN_faces[j].height/2), 0, 0, 360, cv::Scalar( 0, 0, 255 ), 2, 8, 0 );

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first  = center.x;
		newDetection->first.second = center.y;
		newDetection->second       = WHA;
		detection_list_.push_back(newDetection);



	}
//	std::cout << "Finished Searching for Objects"<< std::endl;
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
