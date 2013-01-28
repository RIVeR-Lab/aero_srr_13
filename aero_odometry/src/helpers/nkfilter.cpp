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

NKFilter::NKFilter():
sys_pdf_(NULL),
sys_model_(NULL),
filter_init_(false),
filter_(NULL)
{

	ColumnVector sys_noise_mu(constants::STATE_SIZE());
	sys_noise_mu(constants::X_STATE())           = constants::MU_SYSTEM_NOISE_X();
	sys_noise_mu(constants::Y_STATE())           = constants::MU_SYSTEM_NOISE_Y();
	//sys_noise_Mu(constants::Z_STATE())         = constants::MU_SYSTEM_NOISE_Z();
	sys_noise_mu(constants::THETA_STATE())       = constants::MU_SYSTEM_NOISE_THETA();
	//sys_noise_Mu(constants::PHI_STATE())       = constants::MU_SYSTEM_NOISE_PHI();
	//sys_noise_Mu(constants::GAMMA_STATE())     = constants::MU_SYSTEM_NOISE_GAMMA();
	sys_noise_mu(constants::X_DOT_STATE())       = constants::MU_SYSTEM_NOISE_X();
	sys_noise_mu(constants::Y_DOT_STATE())       = constants::MU_SYSTEM_NOISE_Y();
	//sys_noise_Mu(constants::Z_DOT_STATE())     = constants::MU_SYSTEM_NOISE_Z();
	sys_noise_mu(constants::THETA_DOT_STATE())   = constants::MU_SYSTEM_NOISE_THETA();
	//sys_noise_Mu(constants::PHI_DOT_STATE())   = constants::MU_SYSTEM_NOISE_PHI();
	//sys_noise_Mu(constants::GAMMA_DOT_STATE()) = constants::MU_SYSTEM_NOISE_GAMMA();

	SymmetricMatrix sys_noise_cov(constants::STATE_SIZE());
	sys_noise_cov = 0.0;
	sys_noise_cov(constants::X_STATE(), constants::X_STATE())   = constants::SIGMA_SYSTEM_NOISE_X();
	sys_noise_cov(constants::Y_STATE(), constants::Y_STATE())   = constants::SIGMA_SYSTEM_NOISE_Y();
	//sys_noise_Cov(constants::Z_STATE(), constants::Z_STATE()) = constants::SIGMA_SYSTEM_NOISE_Z();
	sys_noise_cov(constants::THETA_STATE(), constants::THETA_STATE())   = constants::SIGMA_SYSTEM_NOISE_THETA();
	//sys_noise_Cov(constants::PHI_STATE(), constants::PHI_STATE())     = constants::SIGMA_SYSTEM_NOISE_PHI();
	//sys_noise_Cov(constants::GAMMA_STATE(), constants::GAMMA_STATE()) = constants::SIGMA_SYSTEM_NOISE_GAMMA();
	sys_noise_cov(constants::X_DOT_STATE(), constants::X_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_X_DOT();
	sys_noise_cov(constants::Y_DOT_STATE(), constants::Y_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_Y_DOT();
	//sys_noise_Cov(constants::Z_DOT_STATE(), constants::Z_DOT_STATE()) = constants::SIGMA_SYSTEM_NOISE_Z_DOT();
	sys_noise_cov(constants::THETA_DOT_STATE(), constants::THETA_DOT_STATE())   = constants::SIGMA_SYSTEM_NOISE_THETA_DOT();
	//sys_noise_Cov(constants::PHI_DOT_STATE(), constants::PHI_DOT_STATE())     = constants::SIGMA_SYSTEM_NOISE_PHI_DOT();
	//sys_noise_Cov(constants::GAMMA_DOT_STATE(), constants::GAMMA_DOT_STATE()) = constants::SIGMA_SYSTEM_NOISE_GAMMA_DOT();

	BFL::Gaussian sys_uncertainty(sys_noise_mu, sys_noise_cov);

	this->sys_pdf_   = new BFL::NonLinearAnalyticsContionalGaussianMobile(sys_uncertainty);
	this->sys_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(this->sys_pdf_);
}

NKFilter::~NKFilter()
{
	delete this->prior_;
	delete this->sys_model_;
	delete this->sys_pdf_;
	delete this->filter_;
}

bool NKFilter::init_filter(const ColumnVector& initial_pose_estimate, const SymmetricMatrix& initial_covar_estimate)
{
	this->prior_  = new BFL::Gaussian(initial_pose_estimate, initial_covar_estimate);
	this->filter_ = new BFL::ExtendedKalmanFilter(prior_);
	this->filter_init_ = true;
	return true;
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
