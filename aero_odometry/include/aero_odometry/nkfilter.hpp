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
#include<tf/tf.h>
#include<filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
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
public:
	NKFilter();
	virtual ~NKFilter();

	bool init_filter(const ColumnVector& initial_pose_estimate, const SymmetricMatrix& initial_covar_estimate);

	bool update(vector<bool>active_sensors, const ros::Time& update_time);

	bool getEstimate(tf::Transform& state);
	bool getEstimate(tf::StampedTransform& state);


private:
	BFL::Gaussian                                  *prior_;
	BFL::NonLinearAnalyticsContionalGaussianMobile *sys_pdf_;
	BFL::AnalyticSystemModelGaussianUncertainty    *sys_model_;
	vector< pair< BFL::LinearAnalyticConditionalGaussian*,
	              BFL::LinearAnalyticMeasurementModelGaussianUncertainty*> > measurement_models_;

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

	void tfToStateVector(tf::Transform trans, ColumnVector& result);
	void tfToStateVector(tf::StampedTransform& trans, ColumnVector& result);
	void tfToCovarMatrix(tf::Transform& trans, SymmetricMatrix& result);
	void tfToCovarMatrix(tf::StampedTransform& trans, SymmetricMatrix& result);
};

} /* END NAMESPACE aero_odometry */;
#endif /* NKFILTER_HPP_ */
