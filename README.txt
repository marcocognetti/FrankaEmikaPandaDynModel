authors: Claudio Gaz, Marco Cognetti, Alexander Oliva
date: August 2, 2019

Contacts:
- gaz@diag.uniroma1.it
- marco.cognetti@irisa.fr
- olivalex86@gmail.com (alexander.oliva@inria.fr is no longer available)

This repository contains the code regarding the paper 
C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.

The folder are organized as follows:
1) matlab/
     |__parameters_retrieval/: Matlab code for the parameters retrieval algorithm for the Franka Emika Panda Robot
     |__utils/: A set of useful utility functions
     |__data/: Contain the datasets used in the identification process and other data
     |__generate_joint_traj/: Contains a set of tools for joint trajectory generation
     |__ConvertDynPars2VREP/: Tool that converts the retrieved dynamic parameters according to the V-REP interface
     |__dyn_model_panda/: It contains the dynamical model of the Franka Emika Panda Robot in Matlab
2) V-REP/
     |_models/FrankaEmikaPanda.ttm: the dynamic model of the Franka Emika Panda robot in V-REP
3) pdf/
     |_RA-L_2019_PandaDynIdent_SUPPLEMENTARY_MATERIAL.pdf: this file includes the supplementary material of our paper. It includes the friction parameters, the validation in V-REP and the extracted feasible parameters
4) cpp/
     |__panda_dyn_model_example/ : cpp example that compute the joint torques from the model.
     |__README.txt
     |__Robot_vs_Model_torques.jpg
