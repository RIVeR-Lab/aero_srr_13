#include <object_locator/lower_detector_node.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>
#include "ObjectLocatorParams.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
//#include <opencv2/gpu/stream_accessor.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;
using namespace std;
using namespace cv;
using namespace sensor_msgs;

ImageConverter::ImageConverter() :
		it_(nh_), WINDOWLeft("Left Camera"), WINDOWRight("Right Camera"), WINDOWDisparity(
				"Disparity"), sherlock(.5, .15, .05, .5), gotLeft(false), gotRight(
				false)

{
	//**********enable CUDA*****************
	CUDA_ENABLED = 0;

	//********ROS subscriptions and published topics***************
	ObjLocationPub = nh_.advertise<aero_srr_msgs::ObjectLocationMsg>(
			"ObjectPose", 2);
	image_pub_ = it_.advertise("/out", 1);
	pub_points2_ = nh_.advertise<PointCloud2>("points2", 1);

//	image_left_  = it_.subscribeCamera("/lower_stereo/left/image_rect_color", 1, &ImageConverter::imageCbLeft, this);
//	image_right_ = it_.subscribeCamera("/lower_stereo/right/image_rect_color", 1, &ImageConverter::imageCbRight, this);

//	disp_image_sub_ = nh_.subscribe("/stereo_camera/disparity",1, &ImageConverter::imageCbRight, this);
//	left_rect_sub_ = nh_.subscribe("/stereo_camera/left/image_rect_color",1, &ImageConverter::rectLeftCb, this);
//	right_rect_sub_ = nh_.subscribe("/stereo_camera/right/image_rect_color",1, &ImageConverter::rectRightCb, this);

	image_left_ = it_.subscribeCamera("prosilica/image_raw", 1,
			&ImageConverter::imageCbLeft, this);
	//	image_left_ = it_.subscribeCamera("out", 1, &ImageConverter::imageCbLeft, this);
//	point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2> ("/lower_stereo/points2", 2, &ImageConverter::pointCloudCb, this);
	//********ROS Timer for Disparity image cb**************
	disp_timer = nh_.createTimer(ros::Duration(1 / 18),
			&ImageConverter::computeDisparityCb, this);

	//Cascade Trained xml file locations
	cascade_path_WHA =
			"/home/srr/ObjectDetectionData/exec/cascadeWHAground/cascade.xml";
	cascade_path_PINK =
			"/home/srr/ObjectDetectionData/exec/cascadePINKBALL/cascade.xml";
	cascade_path_WHASUN =
			"/home/srr/ObjectDetectionData/exec/cascadeWHAOutside/cascade.xml";
//	cascade_path_RQT_BALL = "/home/srr/ObjectDetectionData/exec/cascadeWHAOutside/cascade.xml";
	cascade_path_PIPE =
			"/home/srr/ObjectDetectionData/exec/cascadeWHAOutside/cascade.xml";
	ctrLeft = 0;
	ctrRight = 0;
	cv::namedWindow(WINDOWLeft);
	cv::namedWindow(WINDOWRight);
	cv::namedWindow(WINDOWDisparity);
	objset = false;

}

ImageConverter::~ImageConverter() {
	cv::destroyWindow(WINDOWLeft);
	cv::destroyWindow(WINDOWRight);
	cv::destroyWindow(WINDOWDisparity);
	//	cvDestroyAllWindows();
}

void ImageConverter::processImage(const sensor_msgs::Image& msg,
		cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW) {
//	ROS_INFO_STREAM("encoding = " << msg.encoding);
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
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
void ImageConverter::saveImage(const sensor_msgs::Image& msg,
		cv_bridge::CvImagePtr& cv_ptr, int O) {
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat_t img(cv_ptr->image);
	std::stringstream s, d;
	s << "/home/srr/ObjectDetectionData/Stereo/Left/" << ctrLeft << ".png";
//	d << "/home/srr/ObjectDetectionData/Stereo/Right/" << ctrRight<<".png";
	std::cout << s.str() << std::endl;
	std::cout << d.str() << std::endl;
	int c = cv::waitKey(5);
	if (O == 0) {
		if ((char) c == 's') {
			cv::imwrite(s.str(), img);
			ctrLeft++;
		}
	} else {
		if ((char) c == 'd') {
			cv::imwrite(d.str(), img);
			ctrRight++;
		}
	}
}

void ImageConverter::rectLeftCb(const sensor_msgs::ImageConstPtr& msg) {
	left_image = *msg;
	gotLeft = true;
	detectAndDisplay(left_image, mat_left, WINDOWLeft);
}

void ImageConverter::rectRightCb(const sensor_msgs::ImageConstPtr& msg) {
	right_image = *msg;
	gotRight = true;
}
void ImageConverter::imageCbLeft(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& cam_info) {
	left_image = *msg;
	left_info = *cam_info;
	gotLeft = true;
	detectAndDisplay(left_image, mat_left, WINDOWLeft);
//		saveImage(left_image, mat_left,0);

}
void ImageConverter::imageCbRight(const sensor_msgs::ImageConstPtr& msg,
		const sensor_msgs::CameraInfoConstPtr& cam_info) {

	right_image = *msg;
	right_info = *cam_info;
	gotRight = true;
//	saveImage(right_image, mat_right,1);
}

inline bool isValidPoint(const cv::Vec3f& pt) {
	// Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
	// and zero disparities (point mapped to infinity).
	return pt[2] != image_geometry::StereoCameraModel::MISSING_Z
			&& !std::isinf(pt[2]);
}
void ImageConverter::computeDisparity() {
	processImage(left_image, mat_left, WINDOWLeft);
	processImage(right_image, mat_right, WINDOWRight);
//	ROS_INFO_STREAM("Finished acquiring images");
	Mat_t leftRect;
	Mat_t rightRect;
//	leftRect  =  imread("/home/srr/ObjectDetectionData/Tskuba/ALeft.jpg", CV_LOAD_IMAGE_COLOR);
//	rightRect  = imread("/home/srr/ObjectDetectionData/Tskuba/ARight.jpg", CV_LOAD_IMAGE_COLOR);
//	Mat_t img1_rect(leftRect.rows,leftRect.cols,CV_8U);
//	Mat_t img2_rect(leftRect.rows,leftRect.cols,CV_8U);
#ifdef CUDA_ENABLED
	gpu::cvtColor(mat_left->image,img1_rect, CV_BGR2GRAY);
	gpu::cvtColor(mat_right->image,img2_rect, CV_BGR2GRAY);

#else
//	cvtColor(mat_left->image,img1_rect, CV_BGR2GRAY);
//	cvtColor(mat_right->image,img2_rect, CV_BGR2GRAY);
#endif
//	this->stereo_model.updateQ();
	this->stereo_model.fromCameraInfo(this->left_info, this->right_info);
	/**	//	  cv::StereoVar stereo;
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
	/*
	 * gpu::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	 *gpu::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	 *gpu::remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	 *gpu::remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	 */
#else
	/**
	 cv::cvtColor(mat_left->image, left_gray, CV_BGR2GRAY);
	 cv::cvtColor(mat_right->image, right_gray, CV_BGR2GRAY);
	 remap(left_gray,img1_rect, mx1, my1, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	 remap(right_gray,img2_rect,mx2, my2,cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
	 **/
#endif
	cvtColor(mat_left->image, leftRect, CV_BGR2GRAY);
	cvtColor(mat_right->image, rightRect, CV_BGR2GRAY);

	const cv::Mat_<uint8_t> img1_rect(leftRect.rows, leftRect.cols,
			const_cast<uint8_t*>(&leftRect.data[0]), rightRect.step);
	const cv::Mat_<uint8_t> img2_rect(rightRect.rows, rightRect.cols,
			const_cast<uint8_t*>(&rightRect.data[0]), rightRect.step);
// cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
//                            reinterpret_cast<float*>(&disp_msg->image.data[0]),
//                            disp_msg->image.step);

	int heightL = img1_rect.rows;
	int widthL = img1_rect.cols;
	Mat_t disp(heightL, widthL, CV_16S);
//	Mat_t vdisp( heightL, widthL, CV_32FC1 );
//	Mat_t dispn2( heightL, widthL, CV_32F );
	Mat_t dispn(heightL, widthL, CV_32F);
//	Mat_t disp;

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

#ifdef CUDA_ENABLED

	gpu::StereoBM_GPU gpuBM((StereoBM::BASIC_PRESET,numDisp, SADSize));
	gpuBM(img1_rect, img2_rect, disp);

#else
//	cv::StereoSGBM stereoSGBM(minDisp,numDisp, SADSize);
//	cv::StereoSGBM stereoSGBM(minDisp, numDisp, SADSize, P1, P2, disp12MaxDiff, preFilterCap, uniqueness, specSize, specRange, false);
//	stereoSGBM(img1_rect, img2_rect, disp);

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

	cv::Mat_<cv::Vec3f> points_mat_;
	this->stereo_model.projectDisparityImageTo3d(disp, points_mat_, true);
	cv::Mat_<cv::Vec3f> mat = points_mat_;

	sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<
			sensor_msgs::PointCloud2>();
	points_msg->header = left_image.header;
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
	const std::string& encoding = left_image.encoding;
	offset = 0;
	if (encoding == enc::MONO8) {
		const cv::Mat_<uint8_t> color(left_image.height, left_image.width,
				(uint8_t*) &left_image.data[0], left_image.step);
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
		const cv::Mat_<cv::Vec3b> color(left_image.height, left_image.width,
				(cv::Vec3b*) &left_image.data[0], left_image.step);
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
		const cv::Mat_<cv::Vec3b> color(left_image.height, left_image.width,
				(cv::Vec3b*) &left_image.data[0], left_image.step);
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

	pub_points2_.publish(points_msg);

//	std::stringstream s,d;
//		s << "/home/srr/ObjectDetectionData/Disparity1.png";
//		cv::imwrite(s.str(), dispn);
	disp.convertTo(dispn, -1, 1.0 / 16);

	Point2d center;
	center.x = (int) (widthL / 2);
	center.y = (int) (heightL / 2) + 200;
	Point3d center_obj_3d;
	float pre_disp_center = disp.at<float>((int) (heightL / 2),
			(int) (widthL / 2));
	this->stereo_model.projectDisparityTo3d(center, pre_disp_center,
			center_obj_3d);
//	double dispOb = this->stereo_model.getDisparity(1.90515);
//	cout << "Object Disparity should be = "<< dispOb;
	cout << "Center" << endl << " X: " << center_obj_3d.x << endl << "Y: "
			<< center_obj_3d.y << endl << "Z: " << center_obj_3d.z << endl;
	float disp_center = dispn.at<float>((int) (heightL / 2),
			(int) (widthL / 2));
//	    ROS_INFO_STREAM("Pre 16 Disparity Value at center = " << pre_disp_center);
//	    ROS_INFO_STREAM("Disparity Value at center = " << disp_center);
	this->stereo_model.getZ(pre_disp_center);
//	    ROS_INFO_STREAM("Z of disp center = " << pre_disp_center);
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

#endif

	cv::Point3d real_xyz;
	geometry_msgs::PointStamped camera_point, world_point;
	for (int i = 0; i < (int) detection_list_.size(); i++) {
//		cout << endl;
//		cout << "In detection #"<< i+1 << "/"<< detection_list_WHA.size() <<endl;
		Point2d obj_centroid(detection_list_.at(i)->first.first,
				detection_list_.at(i)->first.second);
		Point3d obj_3d;

//		std::cout << "Checking disparity at  " << obj_centroid.x <<","<< obj_centroid.y << std::endl;
//		std::cout << "Range (rows,cols): " << vdisp1.rows <<","<< vdisp1.cols << std::endl;
		if (obj_centroid.x < disp.cols && obj_centroid.y < disp.rows) {
//			cout << "Getting Disparity" <<endl;
			//		float disp_val = dispn.at<float>(obj_centroid.y,obj_centroid.x);
			float disp_val = nNdisp(obj_centroid, disp);
//cout << "Pre Disparity Value of detection "<< disp_val <<endl;			
//cout << "Disparity Value of detection "<< disp_val <<endl;
			this->stereo_model.projectDisparityTo3d(obj_centroid, disp_val,
					obj_3d);
			cout << "Disp: " << disp_val << endl << "X: " << obj_3d.x << endl
					<< "Y: " << obj_3d.y << endl << "Z: " << obj_3d.z << endl;
			tf::Point detection(obj_3d.x, obj_3d.y, obj_3d.z);
//			cout << "adding detection to camera_point" <<endl;
			tf::pointTFToMsg(detection, camera_point.point);
			ros::Time tZero(0);
			camera_point.header.frame_id = "/lower_stereo_optical_frame";
			camera_point.header.stamp = tZero;
			world_point.header.frame_id = "/world";
			world_point.header.stamp = tZero;
//			cout << "Transforming camera to world" <<endl;
			optimus_prime.waitForTransform("/world",
					camera_point.header.frame_id, ros::Time(0),
					ros::Duration(1.0));
			optimus_prime.transformPoint("/world", camera_point, world_point);
//			cout << "Adding TFT to msg" <<endl;
			tf::pointMsgToTF(camera_point.point, detection);
			sherlock.addDetection(detection, detection_list_.at(i)->second);
//			cout << "Added detection to manager" <<endl;
		}

	}

	tf::Point detection;
//	cout << "Clearing list" <<endl;
	detection_list_.clear();
//	cout << "Shrinking Det/ection manager list" <<endl;
	sherlock.shrink();
//	cout << "Finished shrinking list" <<endl;96
	double confidence;
	object_type type;
	if (sherlock.getDetection(detection, type, confidence)) {
		std::string typeString;
		switch (type) {
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
		cout << "I Got A Detection: " << endl << "X:" << detection.getX()
				<< ", Y: " << detection.getY() << ", Z: " << detection.getZ()
				<< ", " << confidence << ", of type: " << typeString
				<< std::endl;
		aero_srr_msgs::ObjectLocationMsg msg;

		msg.header.frame_id = camera_point.header.frame_id;
		msg.header.stamp = ros::Time::now();
		msg.pose.header.frame_id = camera_point.header.frame_id;
		msg.pose.header.stamp = ros::Time::now();
		buildMsg(detection, msg.pose);
		ObjLocationPub.publish(msg);
	}
	Mat_t cmapped;
	disp.convertTo(cmapped, CV_8U);
	cv::ellipse(cmapped, Point2d(detection.getX(), detection.getY()),
			cv::Size(30, 30), 0, 0, 360, 0, 2, 8, 0);
//	cv::ellipse(cmapped, center, cv::Size(20, 20), 0, 0, 360, 255, 2, 8, 0);
	cv::rectangle(frame, Point2d(detection.getX()-100, detection.getY()-100), Point2d(detection.getX()+100, detection.getY()+100),Scalar(255,255,255));
	cv::imshow(WINDOWDisparity, cmapped);
	cv::waitKey(3);

//	cmapped = gray2bgr(vdisp1);

//	cv::imshow(WINDOWLeft, img1_rect );
//	cv::waitKey(3);
//	cv::imshow(WINDOWRight, img2_rect );
//		cv::waitKey(3);

}

float ImageConverter::nNdisp(const Point2d& pt, const Mat_t& disp) {
	int window = 10;
	int startx = pt.x - window;
	int starty = pt.y - window;
	float sum = 0.0;
	for (int i = 0; i < window; i++) {
		for (int j = 0; j < window; j++) {
			float value = disp.at<float>(starty + i, startx + j);
			if (value == -1)
				value = 0.0;
			sum = sum + value;
		}
	}

	return sum / (float) (window * window);
}

Mat_t ImageConverter::gray2bgr(Mat_t img) {
	Mat_t BGR(img.rows, img.cols, CV_32FC3);
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			BGR.at<Vec3i>(i, j)[0] = std::abs(
					sin((img.at<double>(i, j)) * 2.0 * 3.14 + 0.0 * 3.14));
			BGR.at<Vec3i>(i, j)[1] = std::abs(
					sin((img.at<double>(i, j)) * 2.0 * 3.14 + (-0.1) * 3.14));
			BGR.at<Vec3i>(i, j)[2] = std::abs(
					sin((img.at<double>(i, j)) * 2.0 * 3.14 + (-0.3) * 3.14));
			cout << "D:" << img.at<uchar>(i, j) << endl;
			cout << "S:" << img.at<Scalar>(i, j) << endl;
		}
	}

	return BGR;
}

void ImageConverter::buildMsg(const tf::Point& point,
		geometry_msgs::PoseStamped& msg) const {
	tf::pointTFToMsg(point, msg.pose.position);
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	q.normalize();
	tf::quaternionTFToMsg(q, msg.pose.orientation);
}

void ImageConverter::computeDisparityCb(const ros::TimerEvent& event) {
	if (gotLeft && gotRight) {
		computeDisparity();
		gotLeft = false;
		gotRight = false;
	}
//	computeDisparity();
}
void ImageConverter::detectAndDisplay(const sensor_msgs::Image& msg,
		cv_bridge::CvImagePtr& cv_ptr, const char* WINDOW) {

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	frame = cv_ptr->image;
	if (!cascade_WHA.load(cascade_path_WHA)) {
		printf("--(!)Error loading\n");
	}

	if (!cascade_PINK.load(cascade_path_PINK)) {
		printf("--(!)Error loading\n");
	}
	if (!cascade_WHASUN.load(cascade_path_WHASUN)) {
		printf("--(!)Error loading\n");
	}
//	if( !cascade_RQT_BALL.load(cascade_path_RQT_BALL))
//		{
//			printf("--(!)Error loading\n");
//		}
	if (!cascade_PIPE.load(cascade_path_PIPE)) {
		printf("--(!)Error loading\n");
	}
	cv::GaussianBlur(frame, frame, cv::Size(9, 9), 2, 2);

	std::vector<cv::Rect> WHA_faces, PINK_faces, SUN_faces, RQT_faces,
			Pipe_faces;
	std::vector<std::vector<cv::Rect> > Detections;
	int HORIZON = 150;


	Mat_t frame_gray;

	cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
	cv::equalizeHist(frame_gray, frame_gray);
	cv::line(frame,Point2d(0,HORIZON),Point2d(frame_gray.cols,HORIZON),Scalar(0,255,0));

	//-- Detect faces

	cascade_WHA.detectMultiScale(frame_gray, RQT_faces, 1.1, 5, 0,
			cv::Size(30, 39), cv::Size(85, 90)); // works for WHAground !&
	cascade_WHA.detectMultiScale(frame_gray, WHA_faces, 1.1, 5, 0,
			cv::Size(52, 59), cv::Size(85, 90)); // works for WHAground !&
	cascade_PINK.detectMultiScale(frame_gray, PINK_faces, 1.1, 20, 0,
			cv::Size(45, 45), cv::Size(80, 80)); // works for PINK !&
	cascade_WHASUN.detectMultiScale(frame_gray, SUN_faces, 1.1, 20, 0,
			cv::Size(45, 45), cv::Size(80, 80)); // works for WHASUN
	cascade_PIPE.detectMultiScale(frame_gray, Pipe_faces, 1.1, 1, 0,
			cv::Size(30, 70), cv::Size(50, 96)); // works for

	/*
	 * WHA - White hook object inside detection loop
	 */
	for (size_t i = 0; i < WHA_faces.size(); i++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(WHA_faces[i]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(WHA_faces[i].x + WHA_faces[i].width / 2,
				WHA_faces[i].y + WHA_faces[i].height / 2);
		if (center.y > HORIZON) {

			cv::ellipse(frame, center,
					cv::Size(WHA_faces[i].width / 2, WHA_faces[i].height / 2),
					0, 0, 360, cv::Scalar(255, 0, 0), 2, 8, 0);

			//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

			DetectionPtr_t newDetection(new Detection_t());
			newDetection->first.first = center.x;
			newDetection->first.second = center.y;
			newDetection->second = WHA;
			detection_list_.push_back(newDetection);
		}

	}
	/*
	 * PINK_BALL - Tennis ball detection loop
	 */
	for (size_t j = 0; j < PINK_faces.size(); j++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(PINK_faces[j]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(PINK_faces[j].x + PINK_faces[j].width / 2,
				PINK_faces[j].y + PINK_faces[j].height / 2);
		if (center.y > HORIZON) {
		cv::ellipse(frame, center,
				cv::Size(PINK_faces[j].width / 2, PINK_faces[j].height / 2), 0,
				0, 360, cv::Scalar(0, 255, 0), 2, 8, 0);

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = PINK_BALL;
		detection_list_.push_back(newDetection);
		}
	}
	/*
	 * WHA_SUN - White hook object outdoor detection loop
	 */
	for (size_t j = 0; j < SUN_faces.size(); j++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(SUN_faces[j]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(SUN_faces[j].x + SUN_faces[j].width / 2,
				SUN_faces[j].y + SUN_faces[j].height / 2);
		if (center.y > HORIZON) {
		cv::ellipse(frame, center,
				cv::Size(SUN_faces[j].width / 2, SUN_faces[j].height / 2), 0, 0,
				360, cv::Scalar(0, 0, 255), 2, 8, 0);

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = WHA;
		detection_list_.push_back(newDetection);
		}
	}
	/*
	 * RQT_BALL - Raquet ball detection loop
	 */
	for (size_t j = 0; j < RQT_faces.size(); j++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(RQT_faces[j]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(RQT_faces[j].x + RQT_faces[j].width / 2,
				RQT_faces[j].y + RQT_faces[j].height / 2);
		if (center.y > HORIZON) {
		cv::ellipse(frame, center,
				cv::Size(RQT_faces[j].width / 2, RQT_faces[j].height / 2), 0, 0,
				360, cv::Scalar(255, 0, 255), 2, 8, 0);

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = RQT_BALL;
		detection_list_.push_back(newDetection);
		}
	}

	/*
	 * PIPE - Pipe Detection Loop
	 */
	for (size_t j = 0; j < Pipe_faces.size(); j++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(Pipe_faces[j]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(Pipe_faces[j].x + Pipe_faces[j].width / 2,
				Pipe_faces[j].y + Pipe_faces[j].height / 2);
		if (center.y > HORIZON) {
		cv::ellipse(frame, center,
				cv::Size(Pipe_faces[j].width / 2, Pipe_faces[j].height / 2), 0,
				0, 360, cv::Scalar(15, 0, 255), 2, 8, 0);

		cv::rectangle(frame,
				Point(center.x - Pipe_faces[j].width / 2,
						center.y - Pipe_faces[j].height / 2),
				Point(center.x + Pipe_faces[j].width / 2,
						center.y + Pipe_faces[j].height / 2),
				cv::Scalar(15, 0, 255));
		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = WHA;
		detection_list_.push_back(newDetection);
		}
	}
//	std::cout << "Finished Searching for Objects"<< std::endl;
	//-- Show what you got
	cv::imshow(WINDOWLeft, frame);

	cv::waitKey(3);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
