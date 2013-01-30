/**
 * @file	nkfilter.hpp
 * @date	Jan 28, 2013
 * @author	Adam Panzica
 * @brief	Definitions for the n-sensor kalman filter
 */

/*
* LICENSE FILE
*/

#ifndef NKFILTER_HPP_
#define NKFILTER_HPP_

//******************* SYSTEM DEPENDANCIES ****************//
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include<boost/circular_buffer.hpp>
#include<boost/tuple/tuple.hpp>
#include<boost/unordered_map.hpp>
//******************* LOCAL DEPENDANCIES ****************//
#include<aero_odometry/nonlinearanalyticconditionalgaussianmobile.h>
//*********************** NAMESPACES ********************//
using MatrixWrapper::ColumnVector;
using MatrixWrapper::Matrix;
using MatrixWrapper::SymmetricMatrix;
using std::string;
using std::vector;
using std::pair;

namespace aero_odometry
{


class NKFilter{
private:
	typedef vector< pair< BFL::LinearAnalyticConditionalGaussian*, BFL::LinearAnalyticMeasurementModelGaussianUncertainty*> > measurement_analytics;
	typedef boost::unordered_map<int, pair<bool, nav_msgs::OdometryConstPtr > > measurement_data;

public:
	NKFilter();
	virtual ~NKFilter();

	bool init_filter(const ColumnVector& initial_pose_estimate, const SymmetricMatrix& initial_covar_estimate);

	/**
	 * @brief Updates the filter
	 * @return True if the filter successfully updated
	 *
	 * Calling update causes the filter to look at all measurements in the measurement buffer, and will linearly interpolate
	 * all sensor data to the time of the oldest new measurement.
	 */
	bool update();

	/**
	 * @brief Gets the latest state information (as of the last update) from the filter
	 * @param [out] state A MatrixWrapper::ColumnVector to write the state information to
	 * @return True if state successfully retrieved, else false
	 */
	bool getEstimate(ColumnVector& state);

	/**
	 * @brief Gets the latest state information (as of the last update) from the filter in the form of a nav_msgs::Odometry message
	 * @param state A nav_message::Odometry containing all state information
	 * @return true if the state was successfully retried, else false
	 *
	 * The Odometry message's Pose/Twist covariance matrices are used to indicate which fields contain valid state information.
	 * A value of -1 means that state is 'turned off', any other value means it is active.
	 */
	bool getEstimate(nav_msgs::Odometry& state);

	/**
	 * @brief Pushes a new measurement onto the measurement buffer
	 * @param [in] sensor_index The id of the sensor that the measurement is coming from
	 * @param [in] measurement  The measurement to add
	 * @return
	 */
	bool addMeasurement(int sensor_index, nav_msgs::OdometryConstPtr measurement);

private:
	BFL::Gaussian                                  *prior_;
	BFL::NonLinearAnalyticsContionalGaussianMobile *sys_pdf_;
	BFL::AnalyticSystemModelGaussianUncertainty    *sys_model_;
	measurement_analytics                           measurement_models_;


	measurement_data                                measurement_buffer_;
	measurement_data                                measurement_buffer_last_;


	bool                                            filter_init_;
	BFL::ExtendedKalmanFilter                      *filter_;

	ros::Time                                       last_update_time_;
	ColumnVector                                    last_sate_;

	tf::Transformer                                 transformer_;

	/**
	 * @author Adam Panzica
	 * @brief Simple angle clamping formula from 0-2PI around some reference angle
	 * @param [in/out] raw The raw angle to bound, in radians
	 * @param [in]     zero The reference value to zero around, in radians
	 */
	void angle_bound(double& raw, const double& zero) const;

	/**
	 * @Author Adam Panzica
	 * @brief Converts a nav_msgs::Odometry message into a ColumnVector of state data and SymmetricMatrix of covariance data
	 * @param [in]  measurement The measurement data
	 * @param [out] state Resultant state vector
	 * @param [out] covar Resultant covariance matrix
	 */
	void odomToStateVectorAndCovar(nav_msgs::OdometryConstPtr measurement, ColumnVector& state, SymmetricMatrix& covar);

	/**
	 * @Author Adam Panzica
	 * @param [out] message The nav_msgs::Odometry to write the state to
	 * @param [in]  state   The state to use
	 * @param [in]  covar   The covariance of the state
	 */
	void stateToOdom(nav_msgs::Odometry& message, ColumnVector& state, SymmetricMatrix& covar);
};

} /* END NAMESPACE aero_odometry */;
#endif /* NKFILTER_HPP_ */
