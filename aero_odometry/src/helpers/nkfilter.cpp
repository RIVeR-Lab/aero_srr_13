/**
 * @file	nkfilter.cpp
 * @date	Jan 28, 2013
 * @author	parallels
 * @brief	implementation of the n-sensor ekf
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include<aero_odometry/nkfilter.hpp>
//*********************** NAMESPACES ********************//

using namespace aero_odometry;

NKFilter::NKFilter(int number_of_sensors):
										posterior_(NULL),
										sys_pdf_(NULL),
										sys_model_(NULL),
										measurement_models_(number_of_sensors),
										filter_init_(false),
										filter_(NULL),
										sensors_init_(false),
										measurement_buffer_(number_of_sensors),
										measurement_buffer_last_(number_of_sensors)
{

	ColumnVector sys_noise_mu(constants::STATE_SIZE());
	sys_noise_mu(constants::X_STATE())           = constants::MU_SYSTEM_NOISE_X();
	sys_noise_mu(constants::Y_STATE())           = constants::MU_SYSTEM_NOISE_Y();
	//sys_noise_Mu(constants::Z_STATE())         = constants::MU_SYSTEM_NOISE_Z();
	sys_noise_mu(constants::RZ_STATE())       = constants::MU_SYSTEM_NOISE_RZ();
	//sys_noise_Mu(constants::PHI_STATE())       = constants::MU_SYSTEM_NOISE_PHI();
	//sys_noise_Mu(constants::GAMMA_STATE())     = constants::MU_SYSTEM_NOISE_GAMMA();
	sys_noise_mu(constants::X_DOT_STATE())       = constants::MU_SYSTEM_NOISE_X();
	sys_noise_mu(constants::Y_DOT_STATE())       = constants::MU_SYSTEM_NOISE_Y();
	//sys_noise_Mu(constants::Z_DOT_STATE())     = constants::MU_SYSTEM_NOISE_Z();
	sys_noise_mu(constants::RZ_DOT_STATE())   = constants::MU_SYSTEM_NOISE_RZ();
	//sys_noise_Mu(constants::PHI_DOT_STATE())   = constants::MU_SYSTEM_NOISE_PHI();
	//sys_noise_Mu(constants::GAMMA_DOT_STATE()) = constants::MU_SYSTEM_NOISE_GAMMA();

	SymmetricMatrix sys_noise_cov(constants::STATE_SIZE());
	sys_noise_cov = 0.0;
	sys_noise_cov(constants::X_STATE(), constants::X_STATE())   = constants::SIGMA_SYSTEM_NOISE_X();
	sys_noise_cov(constants::Y_STATE(), constants::Y_STATE())   = constants::SIGMA_SYSTEM_NOISE_Y();
	//sys_noise_Cov(constants::Z_STATE(), constants::Z_STATE()) = constants::SIGMA_SYSTEM_NOISE_Z();
	sys_noise_cov(constants::RZ_STATE(), constants::RZ_STATE())   = constants::SIGMA_SYSTEM_NOISE_RZ();
	//sys_noise_Cov(constants::PHI_STATE(), constants::PHI_STATE())     = constants::SIGMA_SYSTEM_NOISE_PHI();
	//sys_noise_Cov(constants::GAMMA_STATE(), constants::GAMMA_STATE()) = constants::SIGMA_SYSTEM_NOISE_GAMMA();
	sys_noise_cov(constants::X_DOT_STATE(), constants::X_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_X_DOT();
	sys_noise_cov(constants::Y_DOT_STATE(), constants::Y_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_Y_DOT();
	//sys_noise_Cov(constants::Z_DOT_STATE(), constants::Z_DOT_STATE()) = constants::SIGMA_SYSTEM_NOISE_Z_DOT();
	sys_noise_cov(constants::RZ_DOT_STATE(), constants::RZ_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_RZ_DOT();
	//sys_noise_Cov(constants::PHI_DOT_STATE(), constants::PHI_DOT_STATE())     = constants::SIGMA_SYSTEM_NOISE_PHI_DOT();
	//sys_noise_Cov(constants::GAMMA_DOT_STATE(), constants::GAMMA_DOT_STATE()) = constants::SIGMA_SYSTEM_NOISE_GAMMA_DOT();

	BFL::Gaussian sys_uncertainty(sys_noise_mu, sys_noise_cov);

	this->sys_pdf_   = new BFL::NonLinearAnalyticsContionalGaussianMobile(sys_uncertainty);
	this->sys_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(this->sys_pdf_);

	for (int i = 0; i < number_of_sensors; ++i)
	{
		this->measurement_buffer_[i].first = false;
		this->measurement_buffer_last_[i].first = false;
	}
}

NKFilter::~NKFilter()
{
	delete this->posterior_;
	delete this->sys_model_;
	delete this->sys_pdf_;
	delete this->filter_;
}

bool NKFilter::initFilter(const ColumnVector& initial_pose_estimate, const SymmetricMatrix& initial_covar_estimate)
{
	if(initial_pose_estimate.size()==constants::STATE_SIZE()&&initial_covar_estimate.size1()==constants::STATE_SIZE())
	{
		this->posterior_   = new BFL::Gaussian(initial_pose_estimate, initial_covar_estimate);

		this->filter_      = new BFL::ExtendedKalmanFilter(posterior_);
		this->filter_init_ = true;
		return true;
	}
	else
	{
		ROS_ERROR("Expected an initial estimate/covar of size %d states, got one of size %d/%d states", constants::STATE_SIZE(), initial_pose_estimate.size(),initial_covar_estimate.size1());
		return false;
	}
}

bool NKFilter::initSensor(int sensor_index, ColumnVector& measurement_noise, nav_msgs::OdometryConstPtr initial_measurement)
{
	//Check to make sure the sensor exists
	if(this->measurement_buffer_last_.count(sensor_index)==1)
	{
		//Add the sensor's initial measurment into measurement_buffer_last_
		this->measurement_buffer_last_[sensor_index].first  = true;
		this->measurement_buffer_last_[sensor_index].second = initial_measurement;

		ColumnVector    i_state(constants::STATE_SIZE());
		SymmetricMatrix i_covar(constants::STATE_SIZE());
		this->odomToStateVectorAndCovar(initial_measurement, i_state, i_covar);
		Matrix H(constants::STATE_SIZE(), constants::STATE_SIZE());
		H = 0;
		for (int i = 1; i<=constants::STATE_SIZE(); ++i) {
			if(i_covar(i,i) > 0)
			{
				H(i,i) = 1;
			}
			else
			{
				i_covar(i,i) = std::numeric_limits<double>::max();
			}
		}



		//Initialize its sensor model
		BFL::Gaussian measurement_uncertanty(measurement_noise, i_covar);
		this->measurement_models_.at(sensor_index).first  = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertanty);
		this->measurement_models_.at(sensor_index).second = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(this->measurement_models_.at(sensor_index).first);

		//Check to see if all expected sensors are initialized
		bool sensor_check = true;
		BOOST_FOREACH(measurement_data::value_type sensor, this->measurement_buffer_last_)
		{
			sensor_check &= sensor.second.first;
		}
		if(sensor_check)
		{
			this->sensors_init_ = true;
		}
		return true;
	}
	else
	{
		ROS_ERROR("Sensor %d could not be initialized because it doesn't exist!", sensor_index);
		return false;
	}
}

bool NKFilter::isInitialized()
{
	return this->sensors_init_&&this->filter_init_;
}

bool NKFilter::update()
{
	//Only run if the filter has been initialized properly
	if(this->isInitialized())
	{
		//Amount of time that has elapsed since last filter update
		ros::Time update_time = ros::Time::now();
		double dt = (update_time - this->last_update_time_).toSec();

		//Check to make sure we're not in the past due to clock de-sync
		if(dt<0)
		{
			ROS_ERROR("Cannot Perform Update %f Seconds In the Past!", dt);
			return false;
		}
		ROS_DEBUG_STREAM("Performing Filter Update at "<<update_time.toSec()<<"s With dt="<<dt<<"s");

		//TODO determine which is the most up to date sensor
		BOOST_FOREACH(measurement_data::value_type measurement, this->measurement_buffer_)
		{
			//TODO Actually perform the update on each sensor
		}

		//Get the new prior data and store it
		this->posterior_ = this->filter_->PostGet();
		return true;
	}
	else
	{
		ROS_ERROR("Cannot Perform Update on Uninitialized Filter/Sensors");
		return false;
	}
}

bool NKFilter::getEstimate(ColumnVector& state, SymmetricMatrix& covar)
{
	//Check to see if there is any prior data, then fill state/covar if there is
	if(this->posterior_!=NULL)
	{
		state = this->posterior_->ExpectedValueGet();
		covar = this->posterior_->CovarianceGet();
		return true;
	}
	else
	{
		ROS_ERROR("Cannot Get Estimate Without Performing an Update First!");
		return false;
	}
}

bool NKFilter::getEstimate(nav_msgs::Odometry& state)
{
	ColumnVector    t_state(constants::STATE_SIZE());
	SymmetricMatrix t_covar(constants::STATE_SIZE());
	if(this->getEstimate(t_state, t_covar))
	{
		this->stateToOdom(state, t_state, t_covar);
		return true;
	}
	else return false;
}


void NKFilter::angle_bound(double& raw, const double& zero) const{
	double diff = raw-zero;
	if(diff > M_PI)
	{
		raw -= 2.0*M_PI;
	}
	else if (diff< 0)
	{
		raw += 2.0*M_PI;
	}
}

void NKFilter::odomToStateVectorAndCovar(nav_msgs::OdometryConstPtr measurement, ColumnVector& state, SymmetricMatrix& covar)
{
	if(state.size() == constants::STATE_SIZE())
	{
		state(constants::X_STATE()) = measurement->pose.pose.position.x;
		state(constants::Y_STATE()) = measurement->pose.pose.position.y;
		state(constants::Z_STATE()) = measurement->pose.pose.position.z;

		tf::Quaternion q;
		tf::quaternionMsgToTF(measurement->pose.pose.orientation, q);
		tf::Matrix3x3 orientation(q);
		double z,y,x;
		orientation.getEulerZYX(z,y,x);

		state(constants::RZ_STATE()) = z;
		state(constants::RX_STATE()) = x;
		state(constants::RY_STATE()) = y;


		state(constants::X_DOT_STATE()) = measurement->twist.twist.linear.x;
		state(constants::Y_DOT_STATE()) = measurement->twist.twist.linear.y;
		state(constants::Z_DOT_STATE()) = measurement->twist.twist.linear.z;
		state(constants::RZ_DOT_STATE()) = measurement->twist.twist.angular.z;
		state(constants::RY_DOT_STATE()) = measurement->twist.twist.angular.y;
		state(constants::RX_DOT_STATE()) = measurement->twist.twist.angular.x;

	}
	else
	{
		ROS_ERROR("Tried to place state size %d into mismatched size vector %d", constants::STATE_SIZE(), state.size());
	}

}

void NKFilter::stateToOdom(nav_msgs::Odometry& message, ColumnVector& state, SymmetricMatrix& covar)
{
	if((state.size()==6)&&(covar.size1()==6))
	{
		message.pose.pose.position.x = state(constants::X_STATE());
		message.pose.pose.position.y = state(constants::Y_STATE());
		message.pose.pose.position.z = state(constants::Z_STATE());
		tf::Quaternion orientation(state(constants::RY_STATE()), state(constants::RX_STATE()), state(constants::RZ_STATE()));
		tf::quaternionTFToMsg(orientation, message.pose.pose.orientation);

		message.twist.twist.linear.x = state(constants::X_DOT_STATE());
		message.twist.twist.linear.y = state(constants::Y_DOT_STATE());
		message.twist.twist.linear.z = state(constants::Z_DOT_STATE());
		message.twist.twist.angular.x= state(constants::RX_DOT_STATE());
		message.twist.twist.angular.y= state(constants::RY_DOT_STATE());
		message.twist.twist.angular.z= state(constants::RZ_DOT_STATE());
	}
	else
	{
		ROS_ERROR("Not set up to convert from state size %d and covar size %d to Odometry", state.size(), covar.size1());
	}
}

void NKFilter::flushBuffer()
{
	BOOST_FOREACH(measurement_data::value_type measurement, this->measurement_buffer_)
					{
		//If there was a measurement, copy it to the old measurement buffer and reset its space in the current measurement buffer
		if(measurement.second.first)
		{
			this->measurement_buffer_last_[measurement.first].second = measurement.second.second;
			measurement.second.first = false;
		}
					}
}
