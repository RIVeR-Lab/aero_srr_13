#include <object_locator/ros2cv.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
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
	image_left_ = it_.subscribeCamera("/stereo_top/left/image_raw", 1, &ImageConverter::imageCbLeft, this);
	image_right_ = it_.subscribeCamera("/stereo_top/right/image_raw", 1, &ImageConverter::imageCbRight, this);
	//	image_left_ = it_.subscribeCamera("prosilica/image_raw", 1, &ImageConverter::imageCbLeft, this);
	//	image_left_ = it_.subscribeCamera("out", 1, &ImageConverter::imageCbLeft, this);

	//********ROS Timer for Disparity image cb**************
	disp_timer = nh_.createTimer(ros::Duration(1/18), &ImageConverter::computeDisparityCb,this);

	//Cascade Trained xml file locations
	cascade_path = "/home/srr/ObjectDetectionData/exec/cascadeHOGWHA/cascade.xml";
	ctr = 0;
	cv::namedWindow(WINDOWLeft);
	cv::namedWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
	objset = false;

	DetectionPtr_t(new Detection_t(0,0));

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
//		std::stringstream s,d;
//		s << "/home/srr/ObjectDetectionData/Stereo/Left/" << ctr<<".png";
//		d << "/home/srr/ObjectDetectionData/Stereo/Right/" << ctr<<".png";
//		std::cout << s.str()<<std::endl;
	Mat_t img(cv_ptr->image);
	//	std::cout << "displaying image"<<std::endl;
	//	    cv::imshow(WINDOW, img);




		  imshow(WINDOW,img);

//			  	   int c = cv::waitKey(10);
//			  	         if( (char)c == 's' ) { cv::imwrite(s.str(), img); ctr++;}
//			  	         if( (char)c == 'd' ) { cv::imwrite(d.str(), img); ctr++;}
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
	detectAndDisplay(left_image,mat_left,WINDOWLeft);
		processImage(left_image, mat_left, WINDOWLeft);

}
void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
	right_image = *msg;
	right_info  = *cam_info;
	gotRight = true;
	processImage(right_image, mat_right, WINDOWRight);
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

#ifdef CUDA_ENABLED
	gpu::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	gpu::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	gpu::remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	gpu::remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

#else
	cv::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	cv::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

#endif

	Mat_t disp(  heightL, widthL, CV_16S );
	Mat_t vdisp( heightL, widthL, CV_8UC1 );
	Mat_t dispn( heightL, widthL, CV_32F );
	int minDisp = -50;      //0         //-128-32;
	int numDisp = 16*21;       //80        //256+80;
	int SADSize = 7;				//10
	int P1 =  8*SADSize*SADSize;
	int P2 = 32*SADSize*SADSize;
	int disp12MaxDiff =  1	; // 1;
	int preFilterCap =   31; //  2;
	int uniqueness = 5;
	int specSize =   1000; //50 //20;   //reduces noise
	int specRange = 31  ;  //5 //1;

#ifdef CUDA_ENABLED

	gpu::StereoBM_GPU gpuBM((StereoBM::BASIC_PRESET,numDisp, SADSize));
	gpuBM(img1_rect, img2_rect, disp);

#else
	cv::StereoSGBM stereoSGBM(minDisp, numDisp, SADSize, P1, P2, disp12MaxDiff, preFilterCap, uniqueness, specSize, specRange, true);
	stereoSGBM(img1_rect, img2_rect, disp);
	//cv::StereoBM stereoBM(StereoBM::BASIC_PRESET,numDisp, SADSize);
	//	stereoBM(img1_rect, img2_rect, disp);

#endif
	//    cv::erode(disp, disp, NULL, 2);
	//    cv::dilate(disp, disp, NULL, 2);

	//    cvConvertScale(disp, dispn, 1.0/16);
	//	 cv::filterSpeckles(disp, 200, 24, 13);
	normalize( disp, vdisp, 0, 256, CV_MINMAX );

	Mat_t vdisp1;
#ifdef CUDA_ENABLED
	gpu::resize(disp, vdisp1,size);
#else
	cv::resize(disp, vdisp1,size);
#endif

	//	cv::imshow(WINDOWLeft, img1_rect);
	//	cv::imshow(WINDOWRight, img2_rect);
	Mat_t point_cloud;


	cv::Point3d real_xyz;
	geometry_msgs::PointStamped camera_point, world_point;
	for(int i = 0; i< (int)detection_list_.size(); i++)
	{
//		cout << endl;
//		cout << "In detection #"<< i+1 << "/"<< detection_list_.size() <<endl;
		Point2d obj_centroid(detection_list_.at(i)->first,detection_list_.at(i)->second);
		Point3d obj_3d;


//		std::cout << "Checking disparity at  " << obj_centroid.x <<","<< obj_centroid.y << std::endl;
//		std::cout << "Range (rows,cols): " << vdisp1.rows <<","<< vdisp1.cols << std::endl;
		if(obj_centroid.x < vdisp1.cols && obj_centroid.y < vdisp1.rows)
		{
//			cout << "Getting Disparity" <<endl;
			int disp_val = vdisp1.at<uchar>(obj_centroid.y,obj_centroid.x);
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

			optimus_prime.transformPoint("/world",camera_point, world_point);
//			cout << "Adding TFT to msg" <<endl;
			tf::pointMsgToTF(world_point.point,detection);
			sherlock.addDetection(detection);
//			cout << "Added detection to manager" <<endl;
		}

	}
	tf::Point detection;
//	cout << "Clearing list" <<endl;
	detection_list_.clear();
	DetectionPtr_t(new Detection_t(0,0));
//	cout << "Shrinking Det/ection manager list" <<endl;
	sherlock.shrink();
//	cout << "Finished shrinking list" <<endl;
	double confidence;
	if(sherlock.getDetection(detection, confidence))
	{
		cout<<"I Got A Detection: "<< endl << "X:" << detection.getX() <<", Y: "<< detection.getY() << ", Z: " << detection.getZ() <<", "<< confidence<<std::endl;
		aero_srr_msgs::ObjectLocationMsg msg;

		msg.header.frame_id = world_point.header.frame_id;
		msg.header.stamp = ros::Time::now();
		msg.pose.header.frame_id = world_point.header.frame_id;
		msg.pose.header.stamp = ros::Time::now();
		buildMsg(detection, msg.pose);
		ObjLocationPub.publish(msg);
	}

	cv::imshow(WINDOWDisparity, vdisp1 );
	cv::waitKey(3);

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
	if (gotLeft && gotRight)
	{
		computeDisparity();
		gotLeft = false;
		gotRight = false;
	}
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
	if( !cascade.load( cascade_path ) )
	{
		printf("--(!)Error loading\n");
	}
	cv::GaussianBlur( frame, frame, cv::Size(9, 9), 2, 2 );

	std::vector<cv::Rect> faces;
	std::vector<cv::Rect> pink;
	Mat_t frame_gray;

	cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
	cv::equalizeHist( frame_gray, frame_gray );

	//-- Detect faces
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 35, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDA
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 15, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDB
	//   cascade.detectMultiScale( frame_gray, faces, 1.1, 30, 0, cv::Size(70, 70), cv::Size(90,90) ); // works for LDD 35
	cascade.detectMultiScale( frame_gray, faces, 1.1, 455, 0, cv::Size(70, 100), cv::Size(150,215) ); // works for WHA


	for( size_t i = 0; i < faces.size(); i++ )
	{
		Mat_t faceROI = frame_gray( faces[i] );

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
		cv::ellipse( frame, center, cv::Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar( 255, 0, 0 ), 2, 8, 0 );
//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		detection_list_.push_back(DetectionPtr_t(new Detection_t(center.x, center.y)));



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
