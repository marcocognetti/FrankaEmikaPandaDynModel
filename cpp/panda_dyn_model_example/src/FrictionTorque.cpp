/*
 * FrictionTorque.cpp
 *
 *  Created on: 24 feb 2019
 *     Authors: Oliva Alexander, Gaz Claudio, Cognetti Marco
 *  
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 */

#include "franka_model.h"

using namespace std;

Eigen::Vector7d Friction(const Eigen::Vector7d &dq)
{

	static Eigen::Vector7d tau_f = Eigen::Vector7d::Zero();

	tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
	tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
	tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
	tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
	tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
	tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
	tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

	return tau_f;
}
