author: Claudio Gaz, Marco Cognetti, Alexander Oliva
date: September 4, 2019

-------------------------------------------------
Panda Dynamic Model - Matlab Version 1.0
-------------------------------------------------

This is the code of the dynamic model of the Franka Emika Panda robot, as presented in the following paper:
C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.

The package contains Matlab code and it has been tested on Matlab 2018b. It is composed by the following files:

- main.m: this is the file to be run from a Matlab shell to execute the algorithm. It is a demo file.
- get_CoriolisMatrix.m contains a function that returns the numerical Coriolis and centrifugal forces matrix C(q,dq) in Real^{7x7} given the joint positions q and velocities dq.
- get_CoriolisMatrix.m contains a function that returns the numerical Coriolis and centrifugal forces vector c(q,dq) = C(q,dq)*dq in Real^7 given the joint positions q and velocities dq.
- get_FrictionTorque.m contains a function that returns the numerical joint friction vector tauF(dq) in Real^7, given the joint velocities dq.
- get_GravityVector.m contains a function that returns the numerical gravity vector g(q) in Real^7, given the joint positions q.
- get_MassMatrix.m contains a function that returns the numerical inertia matrix M(q) in Real^{7x7}, given the joint positions q.
