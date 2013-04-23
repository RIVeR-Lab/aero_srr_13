/*
 * SAMStage.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: srr
 */

#include <object_locator/vision_pipeline/SAMStage.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

PLUGINLIB_DECLARE_CLASS(object_locator, SAMStage, object_locator::SAMStage,
		nodelet::Nodelet)
namespace enc = sensor_msgs::image_encodings;
using namespace object_locator;

//static unsigned char colormap[768] =
//	  { 150, 150, 150,
//	    107, 0, 12,
//	    106, 0, 18,
//	    105, 0, 24,
//	    103, 0, 30,
//	    102, 0, 36,
//	    101, 0, 42,
//	    99, 0, 48,
//	    98, 0, 54,
//	    97, 0, 60,
//	    96, 0, 66,
//	    94, 0, 72,
//	    93, 0, 78,
//	    92, 0, 84,
//	    91, 0, 90,
//	    89, 0, 96,
//	    88, 0, 102,
//	    87, 0, 108,
//	    85, 0, 114,
//	    84, 0, 120,
//	    83, 0, 126,
//	    82, 0, 131,
//	    80, 0, 137,
//	    79, 0, 143,
//	    78, 0, 149,
//	    77, 0, 155,
//	    75, 0, 161,
//	    74, 0, 167,
//	    73, 0, 173,
//	    71, 0, 179,
//	    70, 0, 185,
//	    69, 0, 191,
//	    68, 0, 197,
//	    66, 0, 203,
//	    65, 0, 209,
//	    64, 0, 215,
//	    62, 0, 221,
//	    61, 0, 227,
//	    60, 0, 233,
//	    59, 0, 239,
//	    57, 0, 245,
//	    56, 0, 251,
//	    55, 0, 255,
//	    54, 0, 255,
//	    52, 0, 255,
//	    51, 0, 255,
//	    50, 0, 255,
//	    48, 0, 255,
//	    47, 0, 255,
//	    46, 0, 255,
//	    45, 0, 255,
//	    43, 0, 255,
//	    42, 0, 255,
//	    41, 0, 255,
//	    40, 0, 255,
//	    38, 0, 255,
//	    37, 0, 255,
//	    36, 0, 255,
//	    34, 0, 255,
//	    33, 0, 255,
//	    32, 0, 255,
//	    31, 0, 255,
//	    29, 0, 255,
//	    28, 0, 255,
//	    27, 0, 255,
//	    26, 0, 255,
//	    24, 0, 255,
//	    23, 0, 255,
//	    22, 0, 255,
//	    20, 0, 255,
//	    19, 0, 255,
//	    18, 0, 255,
//	    17, 0, 255,
//	    15, 0, 255,
//	    14, 0, 255,
//	    13, 0, 255,
//	    11, 0, 255,
//	    10, 0, 255,
//	    9, 0, 255,
//	    8, 0, 255,
//	    6, 0, 255,
//	    5, 0, 255,
//	    4, 0, 255,
//	    3, 0, 255,
//	    1, 0, 255,
//	    0, 4, 255,
//	    0, 10, 255,
//	    0, 16, 255,
//	    0, 22, 255,
//	    0, 28, 255,
//	    0, 34, 255,
//	    0, 40, 255,
//	    0, 46, 255,
//	    0, 52, 255,
//	    0, 58, 255,
//	    0, 64, 255,
//	    0, 70, 255,
//	    0, 76, 255,
//	    0, 82, 255,
//	    0, 88, 255,
//	    0, 94, 255,
//	    0, 100, 255,
//	    0, 106, 255,
//	    0, 112, 255,
//	    0, 118, 255,
//	    0, 124, 255,
//	    0, 129, 255,
//	    0, 135, 255,
//	    0, 141, 255,
//	    0, 147, 255,
//	    0, 153, 255,
//	    0, 159, 255,
//	    0, 165, 255,
//	    0, 171, 255,
//	    0, 177, 255,
//	    0, 183, 255,
//	    0, 189, 255,
//	    0, 195, 255,
//	    0, 201, 255,
//	    0, 207, 255,
//	    0, 213, 255,
//	    0, 219, 255,
//	    0, 225, 255,
//	    0, 231, 255,
//	    0, 237, 255,
//	    0, 243, 255,
//	    0, 249, 255,
//	    0, 255, 255,
//	    0, 255, 249,
//	    0, 255, 243,
//	    0, 255, 237,
//	    0, 255, 231,
//	    0, 255, 225,
//	    0, 255, 219,
//	    0, 255, 213,
//	    0, 255, 207,
//	    0, 255, 201,
//	    0, 255, 195,
//	    0, 255, 189,
//	    0, 255, 183,
//	    0, 255, 177,
//	    0, 255, 171,
//	    0, 255, 165,
//	    0, 255, 159,
//	    0, 255, 153,
//	    0, 255, 147,
//	    0, 255, 141,
//	    0, 255, 135,
//	    0, 255, 129,
//	    0, 255, 124,
//	    0, 255, 118,
//	    0, 255, 112,
//	    0, 255, 106,
//	    0, 255, 100,
//	    0, 255, 94,
//	    0, 255, 88,
//	    0, 255, 82,
//	    0, 255, 76,
//	    0, 255, 70,
//	    0, 255, 64,
//	    0, 255, 58,
//	    0, 255, 52,
//	    0, 255, 46,
//	    0, 255, 40,
//	    0, 255, 34,
//	    0, 255, 28,
//	    0, 255, 22,
//	    0, 255, 16,
//	    0, 255, 10,
//	    0, 255, 4,
//	    2, 255, 0,
//	    8, 255, 0,
//	    14, 255, 0,
//	    20, 255, 0,
//	    26, 255, 0,
//	    32, 255, 0,
//	    38, 255, 0,
//	    44, 255, 0,
//	    50, 255, 0,
//	    56, 255, 0,
//	    62, 255, 0,
//	    68, 255, 0,
//	    74, 255, 0,
//	    80, 255, 0,
//	    86, 255, 0,
//	    92, 255, 0,
//	    98, 255, 0,
//	    104, 255, 0,
//	    110, 255, 0,
//	    116, 255, 0,
//	    122, 255, 0,
//	    128, 255, 0,
//	    133, 255, 0,
//	    139, 255, 0,
//	    145, 255, 0,
//	    151, 255, 0,
//	    157, 255, 0,
//	    163, 255, 0,
//	    169, 255, 0,
//	    175, 255, 0,
//	    181, 255, 0,
//	    187, 255, 0,
//	    193, 255, 0,
//	    199, 255, 0,
//	    205, 255, 0,
//	    211, 255, 0,
//	    217, 255, 0,
//	    223, 255, 0,
//	    229, 255, 0,
//	    235, 255, 0,
//	    241, 255, 0,
//	    247, 255, 0,
//	    253, 255, 0,
//	    255, 251, 0,
//	    255, 245, 0,
//	    255, 239, 0,
//	    255, 233, 0,
//	    255, 227, 0,
//	    255, 221, 0,
//	    255, 215, 0,
//	    255, 209, 0,
//	    255, 203, 0,
//	    255, 197, 0,
//	    255, 191, 0,
//	    255, 185, 0,
//	    255, 179, 0,
//	    255, 173, 0,
//	    255, 167, 0,
//	    255, 161, 0,
//	    255, 155, 0,
//	    255, 149, 0,
//	    255, 143, 0,
//	    255, 137, 0,
//	    255, 131, 0,
//	    255, 126, 0,
//	    255, 120, 0,
//	    255, 114, 0,
//	    255, 108, 0,
//	    255, 102, 0,
//	    255, 96, 0,
//	    255, 90, 0,
//	    255, 84, 0,
//	    255, 78, 0,
//	    255, 72, 0,
//	    255, 66, 0,
//	    255, 60, 0,
//	    255, 54, 0,
//	    255, 48, 0,
//	    255, 42, 0,
//	    255, 36, 0,
//	    255, 30, 0,
//	    255, 24, 0,
//	    255, 18, 0,
//	    255, 12, 0,
//	    255, 6, 0,
//	    255, 0, 0,
//	  };


void SAMStage::onInit()
{
	NODELET_INFO_STREAM("Initializing SAM Stage");
	loadParams();
	NODELET_INFO_STREAM("Parameters Loaded, registering topics...");
	registerTopics();
	NODELET_INFO_STREAM("SAM Stage Initialized");
}

void SAMStage::loadParams() {
	this->input_topic_  = "disparity_stage/disparity";
	this->output_topic_ = "sam_stage/detection_xyz";
	NODELET_INFO_STREAM("Topics Strings Set");
	this->getPrivateNodeHandle().getParam(this->input_topic_,
			this->input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,
			this->output_topic_);
	NODELET_INFO_STREAM("Topics Set");
	std::string cascade_path_WHA("cascade_path_WHA");
	cascade_path_WHA_ =
			"/home/srr/ObjectDetectionData/exec/cascadeWHAground/cascade.xml";
	this->getPrivateNodeHandle().getParam(cascade_path_WHA, cascade_path_WHA_);

	std::string cascade_path_PINK("cascade_path_PINK");
	cascade_path_PINK_ =
			"/home/srr/ObjectDetectionData/exec/cascadePINKBALL/cascade.xml";
	this->getPrivateNodeHandle().getParam(cascade_path_PINK,
			cascade_path_PINK_);

	cascade_path_WHASUN_ =
			"/home/srr/ObjectDetectionData/exec/cascadeWHAOutside/cascade.xml";
	NODELET_INFO_STREAM("cascade paths set");
	if (!cascade_WHA_.load(cascade_path_WHA_)) {
		NODELET_ERROR_STREAM("--(!)Error loading "<< cascade_path_WHA);
	}

	if (!cascade_PINK_.load(cascade_path_PINK_)) {
		NODELET_ERROR_STREAM("--(!)Error loading " << cascade_path_PINK_);
	}
	if (!cascade_WHASUN_.load(cascade_path_WHASUN_)) {
		NODELET_ERROR_STREAM("--(!)Error loading " << cascade_path_WHASUN_);
	}
	NODELET_INFO_STREAM("cascades loaded");
	std::string thresh_dist("thresh_dist");
	thresh_dist_ = .5;
	this->getPrivateNodeHandle().getParam(thresh_dist, thresh_dist_);

	std::string growth_rate("growth_rate");
	growth_rate_ = .15;
	this->getPrivateNodeHandle().getParam(growth_rate, growth_rate_);

	std::string shrink_rate("shrink_rate");
	shrink_rate_ = .05;
	this->getPrivateNodeHandle().getParam(shrink_rate, shrink_rate_);

	std::string thresh_det("thresh_det");
	thresh_det_ = .5;
	this->getPrivateNodeHandle().getParam(thresh_det, thresh_det_);
	NODELET_INFO_STREAM("Det man vals set");
	this->sherlock_ = new DetectionManager(thresh_dist_, growth_rate_, shrink_rate_, thresh_det_);
	NODELET_INFO_STREAM("Det man Initialized!");
	WINDOWLeft_ = "Left camera image";
//	WINDOWDisp_ = "Disparity image";
	cv::namedWindow(WINDOWLeft_);
//	cv::namedWindow(WINDOWDisp_);
	NODELET_INFO_STREAM("CV windows set.");

}

void SAMStage::registerTopics() {
	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,
			2, &SAMStage::recieveImageCb, this);
	NODELET_INFO_STREAM("registering publisher");
	this->ObjLocationPub_ = this->getNodeHandle().advertise<
			aero_srr_msgs::ObjectLocationMsg>(this->output_topic_, 2);

}

void SAMStage::recieveImageCb(
		const object_locator::SyncImagesAndDisparityConstPtr& msg) {
	aero_srr_msgs::ObjectLocationMsgPtr ptMsg(new aero_srr_msgs::ObjectLocationMsg);
	fetchAndRetrieve(msg->images.left_image);
	calculate3DPoint(msg);
	generateTFMessage(*ptMsg);
	ObjLocationPub_.publish(ptMsg);

}

void SAMStage::fetchAndRetrieve(const sensor_msgs::Image& msg) {
	cv_bridge::CvImagePtr img;
	Mat_t frame_gray;
	std::vector<cv::Rect> WHA_faces, PINK_faces, SUN_faces;
	std::vector<std::vector<cv::Rect> > Detections;

	try {
		img = cv_bridge::toCvCopy(msg, enc::BGR8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::GaussianBlur(img->image, img->image, cv::Size(9, 9), 2, 2);
	cv::cvtColor(img->image, frame_gray, CV_BGR2GRAY);
	cv::equalizeHist(frame_gray, frame_gray);

	cascade_WHA_.detectMultiScale(frame_gray, WHA_faces, 1.1, 5, 0,
			cv::Size(52, 59), cv::Size(85, 90)); // works for WHAground !&
	cascade_PINK_.detectMultiScale(frame_gray, PINK_faces, 1.1, 20, 0,
			cv::Size(45, 45), cv::Size(80, 80)); // works for PINK !&
	cascade_WHASUN_.detectMultiScale(frame_gray, SUN_faces, 1.1, 20, 0,
			cv::Size(45, 45), cv::Size(80, 80)); // works for PINK !&

	for (size_t i = 0; i < WHA_faces.size(); i++) {
		cv::Point center(WHA_faces[i].x + WHA_faces[i].width / 2,
				WHA_faces[i].y + WHA_faces[i].height / 2);

		cv::ellipse(img->image, center,
				cv::Size(WHA_faces[i].width / 2, WHA_faces[i].height / 2), 0, 0,
				360, cv::Scalar(255, 0, 0), 2, 8, 0);

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = WHA;
		detection_list_.push_back(newDetection);
	}
	for (size_t j = 0; j < PINK_faces.size(); j++) {
//		cout << "Entered circle drawing loop" << endl;

		Mat_t faceROI = frame_gray(PINK_faces[j]);

		//-- In each face, detect eyes

		//-- Draw the face
		cv::Point center(PINK_faces[j].x + PINK_faces[j].width / 2,
				PINK_faces[j].y + PINK_faces[j].height / 2);
		cv::ellipse(img->image, center,
				cv::Size(PINK_faces[j].width / 2, PINK_faces[j].height / 2), 0,
				0, 360, cv::Scalar(0, 255, 0), 2, 8, 0);

		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;

		DetectionPtr_t newDetection(new Detection_t());
		newDetection->first.first = center.x;
		newDetection->first.second = center.y;
		newDetection->second = PINK_BALL;
		detection_list_.push_back(newDetection);

	}
//	for (size_t j = 0; j < SUN_faces.size(); j++) {
////		cout << "Entered circle drawing loop" << endl;
//
//		Mat_t faceROI = frame_gray(SUN_faces[j]);
//
//		//-- In each face, detect eyes
//
//		//-- Draw the face
//		cv::Point center(SUN_faces[j].x + SUN_faces[j].width / 2,
//				SUN_faces[j].y + SUN_faces[j].height / 2);
//		cv::ellipse(img->image, center,
//				cv::Size(SUN_faces[j].width / 2, SUN_faces[j].height / 2), 0, 0,
//				360, cv::Scalar(0, 0, 255), 2, 8, 0);
//
//		//		std::cout << "Found object at " << center.x <<","<<center.y<< std::endl;
//
//		DetectionPtr_t newDetection(new Detection_t());
//		newDetection->first.first = center.x;
//		newDetection->first.second = center.y;
//		newDetection->second = WHA;
//		detection_list_.push_back(newDetection);
//
//
//
//	}
	cv::imshow(WINDOWLeft_, img->image);
	cv::waitKey(3);
}

void SAMStage::calculate3DPoint(const object_locator::SyncImagesAndDisparityConstPtr& msg) {
	cv_bridge::CvImagePtr disp;
	try {
		disp = cv_bridge::toCvCopy(msg->disparity_image.image, msg->disparity_image.image.encoding);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	float min_disparity = msg->disparity_image.min_disparity;
	float max_disparity = msg->disparity_image.max_disparity;
	float multiplier = 255.0f / (max_disparity - min_disparity);

	assert(msg->disparity_image.image.encoding == enc::TYPE_32FC1);
//	const cv::Mat_<float> dmat(msg->disparity_image.image.height, msg->disparity_image.image.width,
//			(float*)&msg->disparity_image.image.data[0], msg->disparity_image.image.step);

//	disparity_color_.create(msg->disparity_image.image.height, msg->disparity_image.image.width);
//
//	for (int row = 0; row < disparity_color_.rows; ++row) {
//		const float* d = dmat[row];
//		 for (int col = 0; col < disparity_color_.cols; ++col) {
//			 int index = (d[col] - min_disparity) * multiplier + 0.5;
//			 index = std::min(255, std::max(0, index));
//			 disparity_color_(row, col)[2] = colormap[3*index + 0];
//			 disparity_color_(row, col)[1] = colormap[3*index + 1];
//			 disparity_color_(row, col)[0] = colormap[3*index + 2];
//		 }
//	}
	Mat_t dmat;
	dmat = disp->image;
	Mat_t preDisp(dmat);
	cv::Size ksize;
	ksize.width = 10;
	ksize.height = 10;
	cv::boxFilter(preDisp, dmat, 1, ksize);
//	normalize( preDisp, dmat, 0, 256, CV_MINMAX );
	cv::imshow("disparity", dmat);
//	cv::imshow(WINDOWDisp_, disp->image);
	cv::waitKey(3);
	this->stereo_model_.fromCameraInfo(msg->images.left_info, msg->images.right_info);
	for (int i = 0; i < (int) detection_list_.size(); i++) {
		//		cout << endl;
		//		cout << "In detection #"<< i+1 << "/"<< detection_list_WHA.size() <<endl;
		cv::Point2d obj_centroid(detection_list_.at(i)->first.first,
				detection_list_.at(i)->first.second);
		cv::Point3d obj_3d;

		if (obj_centroid.x < disp->image.cols
				&& obj_centroid.y < disp->image.rows) {
			int disp_val = dmat.at<uchar>(obj_centroid.y,
					obj_centroid.x);
			//			cv::ellipse( vdisp1, obj_centroid, cv::Size( 50, 114), 0, 0, 360, 0, 2, 8, 0 );
			this->stereo_model_.projectDisparityTo3d(obj_centroid, disp_val,
					obj_3d);
			//			cout << "Disp: "<< disp_val << endl << "X: "<< obj_3d.x << endl << "Y: " << obj_3d.y << endl << "Z: " << obj_3d.z << endl;
			tf::Point detection(obj_3d.x, obj_3d.y, obj_3d.z);
			//			cout << "adding detection to camera_point" <<endl;
			tf::pointTFToMsg(detection, camera_point_.point);
			ros::Time tZero(0);
			camera_point_.header.frame_id = "/stereo_bottom/center";
			camera_point_.header.stamp = tZero;
			world_point_.header.frame_id = "/world";
			world_point_.header.stamp = tZero;
			//			cout << "Transforming camera to world" <<endl;
			optimus_prime_.waitForTransform("/world", camera_point_.header.frame_id, ros::Time(0), ros::Duration(1.0));
			optimus_prime_.transformPoint("/world", camera_point_, world_point_);
			//			cout << "Adding TFT to msg" <<endl;
			tf::pointMsgToTF(world_point_.point, detection);
			sherlock_->addDetection(detection, detection_list_.at(i)->second);
			//			cout << "Added detection to manager" <<endl;
		}

	}

//	cout << "Clearing list" <<endl;
	detection_list_.clear();
//	cout << "Shrinking Det/ection manager list" <<endl;
	sherlock_->shrink();
//	cout << "Finished shrinking list" <<endl;
	double confidence;
	object_type type;
	if (sherlock_->getDetection(detection_, type, confidence)) {
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
		NODELET_INFO_STREAM("I Got A Detection: " << std::endl << "X:"
				<< detection_.getX() << ", Y: " << detection_.getY() << ", Z: "
				<< detection_.getZ() << ", " << confidence << ", of type: "
				<< typeString);

	}
}

void SAMStage::generateTFMessage(aero_srr_msgs::ObjectLocationMsg& msg) const {
	msg.header.frame_id = world_point_.header.frame_id;
	msg.header.stamp = ros::Time::now();
	msg.pose.header.frame_id = world_point_.header.frame_id;
	msg.pose.header.stamp = ros::Time::now();
	buildMsg(detection_, msg.pose);
}
void SAMStage::buildMsg(const tf::Point& point,
		geometry_msgs::PoseStamped& msg) const {
	tf::pointTFToMsg(point, msg.pose.position);
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	q.normalize();
	tf::quaternionTFToMsg(q, msg.pose.orientation);
}

