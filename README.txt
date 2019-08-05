author: Claudio Gaz, Marco Cognetti, Alexander Oliva
date: August 2, 2019

This repository contains the code regarding the paper 
C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.

The folder are organized as follows:
1) matlab
     |__parameters_retrieval: Matlab code for the parameters retrieval algorithm for the Franka Emika Panda Robot
     |__utils: A set of useful utility functions
     |__data: Contain the datasets used in the identification process and other data
     |__generate_joint_traj: Contains a set of tools for joint trajectory generation
     |__convert_dyn_pars_2_vrep: Tool that converts the retrieved dynamic parameters according to the V-REP interface 
