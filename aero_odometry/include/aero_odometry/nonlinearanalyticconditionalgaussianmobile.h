/**
 * @file	nonlinearanalyticconditionalgaussianmobile.h
 * @date	Jan 28, 2013
 * @author	Adam Panzica
 * @brief	Defines the System Model for the N-sensor EKF
 *
 * Portions taken from the tutorials for the BFL library
 */

/*
 * LICENSE FILE
 */

#ifndef NONLINEARANALYTICCONDITIONALGAUSSIANMOBILE_H_
#define NONLINEARANALYTICCONDITIONALGAUSSIANMOBILE_H_

//******************* SYSTEM DEPENDANCIES ****************//
  #include <pdf/analyticconditionalgaussian_additivenoise.h>
//******************* LOCAL DEPENDANCIES ****************//

//*********************** NAMESPACES ********************//
namespace aero_odometry
{
class constants
{
public:
	static int STATE_SIZE(){return 6;};

	static double MU_SYSTEM_NOISE_X(){return 1.0;};
	static double MU_SYSTEM_NOISE_Y(){return 1.0;};
	static double MU_SYSTEM_NOISE_Z(){return 1.0;};
	static double MU_SYSTEM_NOISE_THETA(){return 1.0;};
	static double MU_SYSTEM_NOISE_PHI(){return 1.0;};
	static double MU_SYSTEM_NOISE_GAMMA(){return 1.0;};
	static double MU_SYSTEM_NOISE_X_DOT(){return 1.0;};
	static double MU_SYSTEM_NOISE_Y_DOT(){return 1.0;};
	static double MU_SYSTEM_NOISE_Z_DOT(){return 1.0;};
	static double MU_SYSTEM_NOISE_THETA_DOT(){return 1.0;};
	static double MU_SYSTEM_NOISE_PHI_DOT(){return 1.0;};
	static double MU_SYSTEM_NOISE_GAMMA_DOT(){return 1.0;};

	static double SIGMA_SYSTEM_NOISE_X(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_X_DOT(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_Y(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_Y_DOT(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_Z(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_Z_DOT(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_THETA(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_THETA_DOT(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_PHI(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_PHI_DOT(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_GAMMA(){return 1.0;};
	static double SIGMA_SYSTEM_NOISE_GAMMA_DOT(){return 1.0;};

	static int X_STATE(){return 1;}
	static int Y_STATE(){return 2;}
	static int Z_STATE(){return 3;}
	static int X_DOT_STATE(){return 4;}
	static int Y_DOT_STATE(){return 5;}
	static int Z_DOT_STATE(){return 6;}
	static int THETA_STATE(){return 7;}
	static int PHI_STATE(){return 8;}
	static int GAMMA_STATE(){return 9;}
	static int THETA_DOT_STATE(){return 10;}
	static int PHI_DOT_STATE(){return 11;}
	static int GAMMA_DOT_STATE(){return 12;}

	static int LINEAR_V_INPUT(){return 1;};
	static int OMEGA_INPUT(){return 2;};

};

}; /* NAMESPACE aero_odometry */

namespace BFL
{
/**
 * @author Adam Panzica
 * Implementation of the non-linear system model. Adapted to 12DOF from
 * the BFL library tutorials
 */
class NonLinearAnalyticsContionalGaussianMobile : public AnalyticConditionalGaussianAdditiveNoise
{
public:
	NonLinearAnalyticsContionalGaussianMobile(const Gaussian& additive_noise);
	virtual ~NonLinearAnalyticsContionalGaussianMobile();

	virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
	virtual MatrixWrapper::Matrix       dfGet(unsigned int i) const;

private:
	mutable MatrixWrapper::Matrix df_;	///Used to save a memory allocation that would just be copied anyway
};
}; /* NAMESPACE BFL */


#endif /* NONLINEARANALYTICCONDITIONALGAUSSIANMOBILE_H_ */
