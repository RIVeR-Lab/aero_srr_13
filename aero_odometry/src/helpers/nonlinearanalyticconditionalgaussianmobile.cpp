/**
 * @file	nonlinearanalyticconditionalgaussianmobile.cpp
 * @date	Jan 28, 2013
 * @author	parallels
 * @brief	//TODO fill in detailed discription here
 */

/*
 * LICENSE FILE
 */
//******************* SYSTEM DEPENDANCIES ****************//
//******************* LOCAL DEPENDANCIES ****************//
#include<aero_odometry/nonlinearanalyticconditionalgaussianmobile.h>
//*********************** NAMESPACES ********************//

namespace BFL{
using MatrixWrapper::ColumnVector;
using MatrixWrapper::Matrix;
using aero_odometry::constants;

NonLinearAnalyticsContionalGaussianMobile::NonLinearAnalyticsContionalGaussianMobile(const Gaussian& additive_noise):
			df_(constants::STATE_SIZE(),constants::STATE_SIZE()){
	for(int r=1; r<=constants::STATE_SIZE(); r++)
	{
		for(int c=1; c<=constants::STATE_SIZE(); c++)
		{
			if(r==c)
			{
				this->df_(r,c) = 1;
			}
			else
			{
				this->df_(r,c) = 0;
			}
		}
	}
}

ColumnVector NonLinearAnalyticsContionalGaussianMobile::ExpectedValueGet() const
{
	ColumnVector state = ConditionalArgumentGet(0);
	ColumnVector input = ConditionalArgumentGet(1);

	/*
	 * This is essentially combining the standard linearized x[k] = A(x)*x[k-1]+B(x)*u[k] into single equation
	 */

	state(constants::X_STATE())        += cos(state(constants::THETA_STATE()))*input(constants::LINEAR_V_INPUT());
	state(constants::Y_STATE())        += sin(state(constants::THETA_STATE()))*input(constants::LINEAR_V_INPUT());
	//state(constants::Z_STATE())        += 0;
	state(constants::THETA_STATE())    += input(constants::OMEGA_INPUT());
	//state(constants::PHI_STATE())      += 0;
	//state(constants::GAMMA_STATE())    += 0;
	state(constants::X_DOT_STATE())     = cos(state(constants::THETA_STATE()))*input(constants::LINEAR_V_INPUT());
	state(constants::Y_DOT_STATE())     = sin(state(constants::THETA_STATE()))*input(constants::LINEAR_V_INPUT());
	//state(constants::Z_DOT_STATE())     = 0;
	state(constants::THETA_DOT_STATE()) = input(constants::OMEGA_INPUT());
	//state(constants::PHI_DOT_STATE())   = 0;
	//state(constants::GAMMA_DOT_STATE()) = 0;

	return state+AdditiveNoiseMuGet();
}

Matrix NonLinearAnalyticsContionalGaussianMobile::dfGet(unsigned int i) const
{
	ColumnVector state = ConditionalArgumentGet(0);
	ColumnVector input = ConditionalArgumentGet(1);

	/*
	 * This is the state Jacobian
	 */
	this->df_(constants::X_STATE(),constants::X_STATE()) = 1;
	this->df_(constants::Y_STATE(),constants::Y_STATE()) = 1;
	//this->df_(constants::Z_STATE(),constants::Z_STATE()) = 1;
	this->df_(constants::THETA_STATE(),constants::THETA_STATE()) = 1;
	//this->df_(constants::PHI_STATE(),constants::PHI_STATE()) = 1;
	//this->df_(constants::GAMMA_STATE(),constants::GAMMA_STATE()) = 1;

	this->df_(constants::X_STATE(), constants::THETA_STATE()) = -input(constants::LINEAR_V_INPUT())*sin(state(constants::THETA_STATE()));
	this->df_(constants::Y_STATE(), constants::THETA_STATE()) =  input(constants::LINEAR_V_INPUT())*cos(state(constants::THETA_STATE()));

	return this->df_;
}
} /*END NAMESPACE BFL */
