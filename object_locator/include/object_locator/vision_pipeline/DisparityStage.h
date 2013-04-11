/*
 * DisparityStage.h
 *
 *  Created on: Apr 10, 2013
 *      Author: srr
 */

#ifndef DISPARITYSTAGE_H_
#define DISPARITYSTAGE_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <object_locator/SyncImageMsg.h>
#include <object_locator/SyncImagesAndDisparity.h>
#include <object_locator/typedefinitions.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <object_locator/DisparityStageConfig.h>

namespace object_locator
{
	class DisparityStage:public nodelet::Nodelet
	{
	public:
		virtual void onInit();
	protected:
		void loadParams();
		void registerTopics();

		virtual void recieveImageCb(const object_locator::SyncImageMsgConstPtr& msg)const;

		/**
		 * @author Samir Zutshi
		 * @author Adam Panzica
		 * @brief Given an image and camera info computes a rectified image.
		 * @param [in] msg
		 * @param [in] cam_info
		 * @param [out] rectified_img of type CV_8U
		 */
		virtual void computeRectifiedImage(const sensor_msgs::Image& msg, const sensor_msgs::CameraInfo& cam_info, Mat_t& rectified_img)const;

		/**
		 * @author Samir Zutshi
		 * @author Adam Panzica
		 * @brief Given two rectified images, a disparity map is produced
		 * @param [in] rectLeft of type CV_8U
		 * @param [in] rectRight of type CV_8U
		 * @param [out] disparity of type CV_16S
		 */
		virtual void computeDisparity(const Mat_t& rectLeft, const Mat_t& rectRight, Mat_t& disparity)const;

		/**
		 * @author Samir Zutshi
		 * @author Adam Panzica
		 * @brief Builds disparity msg
		 * @param [in] raw_imgs
		 * @param [in] disparity of type CV_16S
		 * @param [out] msg
		 */
		virtual void generateDispMsg(const object_locator::SyncImageMsg& raw_imgs, Mat_t& disparity, object_locator::SyncImagesAndDisparity& msg)const;


		/**
		 * @author Adam Panzica
		 * @brief Callback for handling dynamic_reconfigure requests
		 * @param config
		 * @param level
		 */
		virtual void drCB(object_locator::DisparityStageConfig &config, uint32_t level);

		ros::Subscriber sync_image_sub_;
		ros::Publisher  disp_image_pub_;

		dynamic_reconfigure::Server<object_locator::DisparityStageConfig> dr_server_;

		std::string input_topic_, output_topic_;
		int minDisp_;
		int numDisp_;
		int SADSize_;
		int P1_ ;
		int P2_ ;
		int disp12MaxDiff_;
		int preFilterCap_;
		int uniqueness_;
		int specSize_;
		int specRange_;
		bool fullDp_;
	};
}


#endif /* DISPARITYSTAGE_H_ */
