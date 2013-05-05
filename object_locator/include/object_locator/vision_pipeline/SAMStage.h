/*
 * SAMStage.h
 *
 *  Created on: Apr 11, 2013
 *      Author: srr
 */

#ifndef SAMSTAGE_H_
#define SAMSTAGE_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <object_locator/SyncImagesAndDisparity.h>
#include <object_locator/typedefinitions.h>
#include <object_locator/DetectionManager.h>
#include <aero_srr_msgs/ObjectLocationMsg.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>

namespace object_locator
{
	class SAMStage:public nodelet::Nodelet
	{

	public:
		void onInit();
	protected:
		void loadParams();
		void registerTopics();
		/**
		 * @author Samir Zutshi
		 * @brief Recieves a image message with a stereo pair and disparity image
		 * @param [in] msg
		 * @param [out] image raw image to be processed
		 */
		virtual void recieveImageCb(const object_locator::SyncImagesAndDisparityConstPtr& msg);

		/**
		 * @author Samir Zutshi
		 * @brief Using trained xml data, scans and reports detections to a detection manager
		 * @param [in] image raw image
		 */
		virtual void fetchAndRetrieve(const sensor_msgs::Image& img);

		/**
		 * @author Samir Zutshi
		 * @brief Takes in a disparity image of CV_16S and calculates a point in xyz-space. Then based
		 * 		  on a list of found detections, it pushes out the one with most confidence after a tolerance is reached.
		 * @param [in] disparity image of type CV_16S
		 */
		virtual void calculate3DPoint(const object_locator::SyncImagesAndDisparityConstPtr& msg);

		/**
		 * @author Samir Zutshi
		 * @brief Creates a TF message with pose to publish to arm.
		 * @param [out] msg
		 */
		virtual void generateTFMessage(aero_srr_msgs::ObjectLocationMsg& msg) const;
		virtual void buildMsg(const tf::Point& point, geometry_msgs::PoseStamped& msg) const;
		//********Topic Specific***********************************//
		std::string input_topic_, output_topic_;
		ros::Subscriber sync_image_sub_;
		ros::Publisher ObjLocationPub_;


		//********Cascade Classifier Specific Variables************//
		std::string cascade_path_WHA_,
					cascade_path_PINK_,
					cascade_path_WHASUN_,
					WINDOWLeft_, WINDOWDisparity_;
		CascadeClassifier_t cascade_WHA_, cascade_PINK_, cascade_WHASUN_;

		//********TF point related variables**********************//
		tf::TransformListener optimus_prime_;
		image_geometry::StereoCameraModel stereo_model_;

		//********Detection list and Manager Specific Variables***//
		object_locator::DetectionManager* sherlock_;
		typedef std::pair<int, int> PixPoint_t;
		typedef std::pair<PixPoint_t, object_type> Detection_t;
		typedef boost::shared_ptr<Detection_t> DetectionPtr_t;
		std::vector<DetectionPtr_t> detection_list_;
		geometry_msgs::PointStamped camera_point_, world_point_;
		tf::Point detection_;
		//***sherlock Parameters*****/
		double thresh_dist_, growth_rate_, shrink_rate_, thresh_det_;


		cv::Mat_<cv::Vec3b> disparity_color_;

	};
}


#endif /* SAMSTAGE_H_ */
