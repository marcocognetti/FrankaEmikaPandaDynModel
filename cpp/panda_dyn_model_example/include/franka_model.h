/*
 * franka_model.h
 *
 *  Created on: 24 feb 2019
 *     Authors: Oliva Alexander, Gaz Claudio, Cognetti Marco
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 */


#include <Eigen/Core>

#ifndef INCLUDE_FRANKA_MODEL_H_
#define INCLUDE_FRANKA_MODEL_H_

#define NUMBER_OF_JOINTS 7

namespace Eigen {

	typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , NUMBER_OF_JOINTS > Matrix7d;
	typedef Eigen::Matrix< double , NUMBER_OF_JOINTS , 1 > Vector7d;
}

using namespace std;

/*
 * In order to speed up the code execution, we pre-compute the constant part of the
 * friction model of each joint i (second term of the right member of the equation).
 *
 * tau_f(i) = FI_1(i)/(1+exp(-FI_2(i)*(dq(i)+FI_3(i)))) -FI_1(i)/(1+exp(-FI_2(i)*FI_3(i)))
 *
 * For further information refer to our paper and relative Supplementary Material:
 *      C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,
 * 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of
 *   Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 */

const double  FI_11 = 0.54615;
const double  FI_12 = 0.87224;
const double  FI_13 = 0.64068;
const double  FI_14 = 1.2794;
const double  FI_15 = 0.83904;
const double  FI_16 = 0.30301;
const double  FI_17 = 0.56489;

const double  FI_21 = 5.1181;
const double  FI_22 = 9.0657;
const double  FI_23 = 10.136;
const double  FI_24 = 5.5903;
const double  FI_25 = 8.3469;
const double  FI_26 = 17.133;
const double  FI_27 = 10.336;

const double  FI_31 = 0.039533;
const double  FI_32 = 0.025882;
const double  FI_33 = -0.04607;
const double  FI_34 = 0.036194;
const double  FI_35 = 0.026226;
const double  FI_36 = -0.021047;
const double  FI_37 = 0.0035526;

const double TAU_F_CONST_1 = FI_11/(1+exp(-FI_21*FI_31));
const double TAU_F_CONST_2 = FI_12/(1+exp(-FI_22*FI_32));
const double TAU_F_CONST_3 = FI_13/(1+exp(-FI_23*FI_33));
const double TAU_F_CONST_4 = FI_14/(1+exp(-FI_24*FI_34));
const double TAU_F_CONST_5 = FI_15/(1+exp(-FI_25*FI_35));
const double TAU_F_CONST_6 = FI_16/(1+exp(-FI_26*FI_36));
const double TAU_F_CONST_7 = FI_17/(1+exp(-FI_27*FI_37));


Eigen::Matrix7d MassMatrix(const Eigen::Vector7d &q);

Eigen::Matrix7d CoriolisMatrix(const Eigen::Vector7d &q,const Eigen::Vector7d &dq);

Eigen::Vector7d GravityVector(const Eigen::Vector7d &q);

Eigen::Vector7d Friction(const Eigen::Vector7d &dq);


#endif /* INCLUDE_FRANKA_MODEL_H_ */
