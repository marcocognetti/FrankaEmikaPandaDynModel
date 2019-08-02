author: Claudio Gaz, Marco Cognetti
date: August 2, 2019
 
-------------------------------------------------
Parameters Retrieval Algorithm
-------------------------------------------------
Version 1.0

This is the code of the Parameters Retrieval Algorithm presented in the following paper:
C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.

The package contains Matlab code and it has been tested on Matlab 2018b. It is composed by the following files:

- main.m: this is the file to be run from a Matlab shell to execute the algorithm.
- read_bounds.m: it has the purpose to retrieve from a csv file the lower and the upper bounds of the dynamic parameters of the robot under study.
- error_fcn_gM_LMI.m: computes the cost function (loss) at each step, when we already have at disposal the dynamic coefficients values pi_hat (two-steps identification).
- error_fcn_gM_LMI_regressor.m: computes the cost function (loss) at each step, when we do not have at disposal the dynamic coefficients values pi_hat (one-step identification).
- check_inertia_condition.m: evaluates the external penalty function due to the violation of the link inertia tensors triangular inequalities.
- get_Panda_coefficients_expanded.m: returns the value of the dynamic coefficients vector, given the values of the single dynamic parameters (with the inertia tensors expressed w.r.t. the CoM frames).
- bounds_gM.csv: contains the lower and upper bounds of the dynamic parameters of the robot under study (excluding friction). In this case, the Franka Emika Panda robot.
- bounds_gM_friction.csv: contains the lower and upper bounds of the dynamic parameters of the robot under study (including friction). In this case, the Franka Emika Panda robot.
- regressor_and_pars_data.mat: a Matlab workspace file containing the stacked evaluated regressor of the Franka Emika Panda robot with exciting trajectories data; it contains also the symbolic dynamic coefficients vector of the Panda. 