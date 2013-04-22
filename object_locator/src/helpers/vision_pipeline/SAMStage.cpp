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


void SAMStage::onInit()
{
	NODELET_INFO_STREAM("Initializing SAM Stage");
	loadParams();
	registerTopics();
	NODELET_INFO_STREAM("SAM Stage Initialized");
}

void SAMStage::loadParams() {
	this->input_topic_  = "disparity_stage/disparity";
	this->output_topic_ = "sam_stage/detection_xyz";
	this->getPrivateNodeHandle().getParam(this->input_topic_,
			this->input_topic_);
	this->getPrivateNodeHandle().getParam(this->output_topic_,
			this->output_topic_);

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

	if (!cascade_WHA_.load(cascade_path_WHA_)) {
		NODELET_ERROR_STREAM("--(!)Error loading "<< cascade_path_WHA);
	}

	if (!cascade_PINK_.load(cascade_path_PINK_)) {
		NODELET_ERROR_STREAM("--(!)Error loading " << cascade_path_PINK_);
	}
	if (!cascade_WHASUN_.load(cascade_path_WHASUN_)) {
		NODELET_ERROR_STREAM("--(!)Error loading " << cascade_path_WHASUN_);
	}
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

	this->sherlock_ = new DetectionManager(thresh_dist_, growth_rate_, shrink_rate_, thresh_det_);
	WINDOWLeft_ = "Left camera image";
	WINDOWDisp_ = "Disparity image";
	cv::namedWindow(WINDOWLeft_);
	cv::namedWindow(WINDOWDisp_);
}

void SAMStage::registerTopics() {
	this->sync_image_sub_ = this->getNodeHandle().subscribe(this->input_topic_,
			2, &SAMStage::recieveImageCb, this);
	this->ObjLocationPub_ = this->getNodeHandle().advertise<
			aero_srr_msgs::ObjectLocationMsg>(this->output_topic_, 2);

}

void SAMStage::recieveImageCb(
		const object_locator::SyncImagesAndDisparityConstPtr& msg) {
	aero_srr_msgs::ObjectLocationMsgPtr ptMsg(new aero_srr_msgs::ObjectLocationMsg);
	fetchAndRetrieve(msg->images.left_image);
	calculate3DPoint(msg->disparity_image, msg);
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

void SAMStage::calculate3DPoint(const sensor_msgs::Image& disparity,
		const object_locator::SyncImagesAndDisparityConstPtr& msg) {
	cv_bridge::CvImagePtr disp;
	try {
		disp = cv_bridge::toCvCopy(msg->disparity_image, enc::MONO8);
	} catch (cv_bridge::Exception& e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::imshow(WINDOWDisp_, disp->image);
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
			int disp_val = disp->image.at<uchar>(obj_centroid.y,
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

