/*
 * Model_example.cpp
 *
 *  Created on: Mar 21, 2020
 *     Authors: Oliva Alexander
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 *  This example shows how to use the functions of the estimated dynamic model
 *  of the Franka Emika Panda robot.
 *  The code, loads the trajectory parameter of one of the exciting trajectories
 *  used in the paper and available in the repository. Once the one trajectory
 *  step is computed, the joint torque is computed from the model and stored
 *  in some files. The output data of the example can be used e.g. to evaluate the
 *  estimated model accuracy with respect to the real joint torques. See the
 *  accompanying image: Robot_vs_Model_torques.jpg
 *
 */

#include <iostream>
#include <chrono>
#include <fstream>
#include <cmath>
#include <string>
#include <unistd.h>
#include <sstream>

#include<franka_model.h>

#include <Eigen/Core>

using namespace std;

int main(int argc, char **argv)
{


	string  ParamsFile = "../../matlab/data/Exciting_Traj/Trajectory_1/gen_params.txt";
	string  InitPosFile = "../../matlab/data/Exciting_Traj/Trajectory_1/gen_q0.txt";
	bool give_file = false;

	for (int i = 1; i < argc; i++) {
		if (string(argv[i]) == "--help" || string(argv[i]) == "-h") {
			cout << argv[0] << " [--help] [-h] [--params myParams.txt] [--initPos myInitPos.txt]"
					<< "\n";
			return EXIT_SUCCESS;
		}
		else if (string(argv[i]) == "--params" && i + 1 < argc){
			ParamsFile = argv[i + 1];
		}
		else if (string(argv[i]) == "--initPos" && i + 1 < argc){
			InitPosFile = argv[i + 1];
		}
	}
	if (ParamsFile == "" || InitPosFile == "") {
		cout << argv[0] << "\n Specify the input file name with the [--params] and/or [--initPos] option" << endl;
		return EXIT_SUCCESS;
	}else{
		cout << "Trajectory Parameters file: " <<  ParamsFile << endl;
		cout << "Trajectory Initial pos file: " <<  InitPosFile << endl;
	}


	//First load the initial joint configuration
	Eigen::Vector7d q0;
	ifstream ifsParams, ifsInitPos;
	string str;
	unsigned int i = 0, j=0;
	cout << "Initial joint configuration: " << endl;
	ifsInitPos.open(InitPosFile,ios::in);
	if (ifsInitPos.is_open()) {
		while(getline(ifsInitPos, str)) {
			istringstream ss(str);
			ss >>  q0[i];
			cout << q0[i] << " , ";
			i++;
		}
		cout << endl;
	}
	ifsInitPos.close();

	unsigned int nJoints = 7;
	double T = 1, Ttrj= 20;
	Eigen::Vector7d A,final_speed;
	A.Zero();
	Eigen::Matrix< double , 7 , 5 > a_k_l;
	Eigen::Matrix< double , 7 , 5 > b_k_l;

	// compute parameters needed to generate the exciting trajectory
	cout << "Loaded trajectory parameters: " << endl;
	str = "";
	i = 0, j=0;
	ifsParams.open(ParamsFile,ios::in);
	if (ifsParams.is_open()) {
		while(getline(ifsParams, str)) {
			istringstream ss(str);
			j=0;
			if(i < 7){
				while(ss >> a_k_l(i,j)){
					cout << a_k_l(i,j) << " , ";
					j++;
				}
			}else
			{
				while(ss >> b_k_l(i-7,j)){
					cout << b_k_l(i-7,j) << " , ";
					j++;
				}
			}
			i++;
			cout << endl;
		}
	}
	ifsParams.close();

	for(int i=0; i<7; i++){
		for(int j=0; j<5; j++){
			A[i] = A[i] + a_k_l(i,j);
		}
	}

	ofstream torqueFile, positionsFile, velocityFile, accelerationFile, timeFile;
	torqueFile.open("exciting_traj_torques.txt",ios::out);
	positionsFile.open("exciting_traj_positions.txt",ios::out);
	velocityFile.open("exciting_traj_velocity.txt",ios::out);
	accelerationFile.open("exciting_traj_acceleration.txt",ios::out);
	timeFile.open("exciting_traj_time.txt",ios::out);

	// uncomment this section if you want to collect also this data into files
	/*
	ofstream gravityFile, massFile, coriolisFile, frictionFile;
	gravityFile.open("exciting_traj_gravity.txt",ios::out);
	massFile.open("exciting_traj_mass.txt",ios::out);
	coriolisFile.open("exciting_traj_coriolis.txt",ios::out);
	frictionFile.open("exciting_traj_friction.txt",ios::out);
	*/

	Eigen::Vector7d  q,dq,dq_old,ddq, tau,gravity, friction;
	Eigen::Matrix7d mass, coriolis;


	chrono::high_resolution_clock::time_point start, current, tic, toc, old;
	chrono::duration<double> duration = chrono::steady_clock::duration::zero();
	chrono::duration<double> delta_t = chrono::steady_clock::duration::zero();
	start = chrono::high_resolution_clock::now();
	current = start;

	q = q0;
	dq_old.Zero();

	double min = 10, max = 0, mean = 0;
	unsigned int counter = 0;
	while(duration.count() <= Ttrj + T){
		old = current;
		current = chrono::high_resolution_clock::now();
		duration = chrono::duration_cast<chrono::duration<double>>(current-start);
		delta_t = chrono::duration_cast<chrono::duration<double>>(current - old);

		// compute the exciting velocity trajectory and compute the joint acceleration and position from it
		for(int i=0;i<nJoints;i++){
			if (duration.count() < T/2) {
				dq(i) = (A[i]/2)*(1-cos((2*M_PI)*duration.count()));
			}
			else if (duration.count() >= T/2 && duration.count() < (Ttrj+T/2)) {
				dq(i) = 0.0;
				for (int j=0; j<5; j++){
					dq(i) =  dq(i)+ a_k_l(i,j)*cos(0.15*M_PI*(j+1)*(duration.count() - T/2)) + b_k_l(i,j)*sin(0.15*M_PI*(j+1)*(duration.count() - T/2));
				}
				final_speed(i) = dq(i);
			}
			else if (duration.count() >= (Ttrj+T/2)) {
				dq(i) = (final_speed(i)/2)*(1-cos((2*M_PI)*(duration.count() - Ttrj)));
			}
			ddq(i) = (dq(i) - dq_old(i))/delta_t.count();
			q(i) = q(i) + dq(i)*delta_t.count();
			dq_old(i) = dq(i);
		}

		//get the model
		tic = chrono::high_resolution_clock::now();

		mass = MassMatrix(q);
		coriolis = CoriolisMatrix(q, dq);
		gravity = GravityVector(q);
		friction = Friction(dq);
		// compute the joint torques from the Lagrangian equation of motion
		tau = mass*ddq + coriolis*dq + gravity + friction;

		toc = chrono::high_resolution_clock::now();

		timeFile << duration.count() << endl;
		for(int k = 0; k < nJoints; k++){
			torqueFile << tau(k) << endl;
			positionsFile << q(k) << endl;
			velocityFile << dq(k) << endl;
			accelerationFile << ddq(k) << endl;

			// uncomment for store this values into files
			/*
			gravityFile << gravity(k) << endl;
			frictionFile << friction(k) << endl;
			for(int l=0;l < nJoints; l++){
				massFile << mass(k,l) << "\t";
				coriolisFile << coriolis(k,l) << "\t";
			}
			massFile << endl;
			coriolisFile << endl;
             */

		}

		// make some performance analysis on the execution time
		delta_t = chrono::duration_cast<chrono::duration<double>>(toc-tic);
//		cout << "delta time: " << delta_t.count() << endl;
		if(delta_t.count() < min){
			min = delta_t.count();
		}
		if(delta_t.count() > max){
			max = delta_t.count();
		}
		mean += delta_t.count();
		counter++;
	}
	cout << "_____ Model execution time [sec] ___" << endl;
	cout << "  worst    /    best    /    mean  \n" << max << " / " << min << " / " << mean/counter << "\n\n";

	torqueFile.close();
	positionsFile.close();
	velocityFile.close();
	accelerationFile.close();
	timeFile.close();

	cout << "Some files were generated. Use them to compare the results of the estimated model with the real robot. \n" << endl;
	cout << "The collected data from the real robot, for this example trajectory, is located in: " << endl;
	cout << "/FrankaEmikaPandaDynModel/matlab/data/Exciting_Traj/Trajectory_1/rbt_log \n" << endl;

	//uncomment if you stored this values in files
	/*
	coriolisFile.close();
	gravityFile.close();
	massFile.close();
	frictionFile.close();
    */

	cout << "The end" << endl;
	return EXIT_SUCCESS;

}

