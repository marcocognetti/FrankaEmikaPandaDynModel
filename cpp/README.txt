author: Alexander Oliva

This is a small example that uses the estimated model parameters of our paper:
  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019. 
  

To Execute the example, you have to build it first.
Note: Library Eigen3 is required to build this example. 

Create a build folder alongside the example's folder (this path)
$ mkdir build 

change directory into it
$ cd build/

configure the project using ccmake for instance
$ ccmake ../panda_dyn_model_example

configure [c] and then generate [g] the project.

now, you can build the example, it will take a while, just the time to grab a coffee!
$ make -j4

Run the example
$ ./Model_example

Enjoy!

Here is an extract of the example's execution:
-----------------------------------------------
/FrankaEmikaPandaDynModel/cpp/build$ ./Model_example 
Trajectory Parameters file: ../../matlab/data/Exciting_Traj/Trajectory_1/gen_params.txt
Trajectory Initial pos file: ../../matlab/data/Exciting_Traj/Trajectory_1/gen_q0.txt
Initial joint configuration: 
-0.9584 , 0.5622 , -1.4576 , -2.2141 , -2.5711 , 3.0661 , -0.164597 , 
Loaded trajectory parameters: 
-0.2031 , 0.1295 , -0.009 , 0.2319 , -0.7598 , 
-0.0699 , -0.538 , -0.2015 , -0.5535 , 0.2352 , 
0.3076 , 0.5864 , -0.4813 , -0.1228 , -0.5273 , 
0.1269 , -0.0253 , -0.2405 , 0.2178 , 0.6984 , 
-0.2773 , -0.0857 , 0.481 , 0.5311 , -0.1639 , 
0.21 , -0.1194 , -0.095 , -0.0964 , -0.0399 , 
-0.2273 , -0.5636 , -0.2099 , 0.5725 , -0.3748 , 
-0.1136 , 0.66 , -0.1858 , 0.1867 , 0.2357 , 
-0.2437 , 0.182 , -0.239 , -0.2647 , 0.4252 , 
0.3898 , -0.0268 , 0.0179 , 0.4767 , 0.2726 , 
0.0401 , -0.1503 , 0.2359 , -0.2022 , 0.3693 , 
0.5491 , 0.1046 , 0.0407 , -0.0432 , 0.2448 , 
0.0692 , -0.6946 , -0.4442 , -0.5749 , -0.0698 , 
0.2023 , -0.257 , -0.501 , 0.0774 , -0.4833 , 
_____ Model execution time [sec] ___
  worst    /    best    /    mean  
0.0003881 / 4.2362e-05 / 5.72956e-05

Some files were generated. Use them to compare the results of the estimated model with the real robot. 

The collected data from the real robot, for this example trajectory, is located in: 
/FrankaEmikaPandaDynModel/matlab/data/Exciting_Traj/Trajectory_1/rbt_log 

The end
------------------------------------------------

As can be seen from the results, the model is computed in less than 0.05 ms.
(Dell Latitude 7490 - Intel CORE i7-8650U - 32Gb RAM ) 
This means that you can use this library to control the robot in real-time (1 ms control loop).


