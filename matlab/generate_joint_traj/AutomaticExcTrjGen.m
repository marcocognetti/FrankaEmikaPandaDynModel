%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This script automatically generates exciting trajectories for each    %
%   robot joint while verifying that the joint position, velocity and     %
%   acceleration limits are respected. A collision check against the      %
%   robot's environment is performed. The trajectory is built in three    %
%   parts, first and last T/2 [s] are a (1-cos(2*pi/T)) function that     %
%   brings the joint velocities from zero to the initial velocities in    %
%   the exciting trajectories and from the final velocities again to zero.%
%   The expression of the exciting trajectory is reported in section II   %
%   of the Supplementary Material of our paper:                           %
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,         %
%'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of%
% Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019. %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;
addpath ../utils ../data

%% Load Robot's DH table and define its Joint limits in accordance with 
%  the specifications: 
%  https://frankaemika.github.io/docs/control_parameters.html#constants

[DH,Convention] = Load_Franka_DH();

% joint limits (deg): (q1..q7)
%       [-166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027 -166.0031] [deg]
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973]'; % [rad]
q_min_ = q_min + ones(7,1)*deg2rad(5);  % give a safety margin of 5 deg
%       [166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024 166.0031] [deg]
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973]'; % [rad]
q_max_ = q_max - ones(7,1)*deg2rad(5);  % give a safety margin of 5 deg
% joint velocity limits (rad/s): (q1..q7)
dq_max = [2.1750 2.1750 2.1750 2.1750 2.6100 2.6100 2.6100]';
dq_max_ = dq_max - ones(7,1)*deg2rad(10);
dq_min = -dq_max;
dq_min_ = -dq_max_;
% joint acceleration limits (rad/s^2): (q1..q7)
ddq_max = [15 7.5 10 12.5 15 20 20]';
ddq_min = -ddq_max; 

%% Set the parameters for the exciting trajectories
wf = 0.15*pi;      % angular frequency
L = 5;             % number of harmonics
N = 7;             % number of joints
Tr = 2;            % number of exciting trajectories to be generated
T = 1;             % total time for velocity rising and falling [s]
dt = 0.01;         % trajectory sampling rate [s]
Trj_duration = 20; % trajectory duration unless T [s]

a_k_l = zeros(N,L,Tr);
b_k_l = zeros(N,L,Tr);

A = zeros(N,1);
B = zeros(N,1);

%% Generation of time dipendent trajectories for each joint

tt = 0:dt:Trj_duration + T;
tr = 1;
while tr <= Tr
    j=1;
    % if a trajectory is not found after 50 iterations, the algorithm
    % automatically decreases this values until a trajectory is built
    a_a = 1;
    b_a = -a_a;
    a_b = 1;
    b_b = -a_b;

    trials = 1;
    
    while j <= N   
        q(j,:,tr) = zeros(1,size(tt,2));
        dq(j,:,tr) = zeros(1,size(tt,2));
        ddq(j,:,tr) = zeros(1,size(tt,2));
        
        next = false;
        % randomly generate the harmonics amplitudes
        a_k_l(j,:,tr) = a_a*ones(1,L) + (b_a -a_a)*rand(1,L);
        b_k_l(j,:,tr) = a_b*ones(1,L) + (b_b -a_b)*rand(1,L);

        %precompute the starting amplitude of the velocities in the central phase

        A(j) = sum(a_k_l(j,:,tr));
        B(j) = sum(-((b_k_l(j,:,tr))./(wf.*(1:L))));

        s = 1;
        [v, f] = min(abs(tt - T/2 - dt));
        t = tt(s:f);

        q_init_end = -(A(j)/(4*pi))*sin(2*pi*(t(end))+dt) + (A(j)/2)*(t(end)+dt);
    
        q(j,s:f,tr) = -((A(j)*T)/(4*pi))*sin(((2*pi)/T)*t) + (A(j)/2)*t;
        dq(j,s:f,tr) = (A(j)/2)*(1-cos(((2*pi)/T)*t));
        ddq(j,s:f,tr) = ((A(j)*pi)/T)*sin(((2*pi)/T)*t);
    
        q(j,s:f,tr) = q(j,s:f,tr) - q(j,f,tr) + B(j);

        s = f+1;
        [~, f] = min(abs(tt - Trj_duration -T/2 -dt));
        t = tt(s:f);


        for l=1:5
           q(j,s:f,tr) = q(j,s:f,tr) + (a_k_l(j,l,tr)/(wf*l))*sin(wf*l*(t-t(1))) - (b_k_l(j,l,tr)/(wf*l))*cos(wf*l*(t-t(1))); 
           dq(j,s:f,tr) = dq(j,s:f,tr) + a_k_l(j,l,tr)*cos(wf*l*(t -t(1))) + b_k_l(j,l,tr)*sin(wf*l*(t-t(1)));
           ddq(j,s:f,tr) = ddq(j,s:f,tr) - a_k_l(j,l,tr)*sin(wf*l*(t-t(1)))*wf*l + b_k_l(j,l,tr)*cos(wf*l*(t-t(1)))*wf*l;
        end 
            
        [~, s] = min(abs(tt - Trj_duration - T/2));
        t = tt(s:end);

        q(j,s:end,tr) = -(dq(j,s-1,tr)/(4*pi)).*sin(2*pi*(t - t(1) + T/2)) + (dq(j,s-1,tr)/2)*(t - t(1) + T/2);
        q(j,s:end,tr) =  q(j,s:end,tr) -q(j,s,tr) + q(j,s-1,tr);
        dq(j,s:end,tr) = (dq(j,s-1,tr)/2).*(1-cos((2*pi)*(t - t(1) + T/2)));
        ddq(j,s:end,tr) = (dq(j,s-1,tr)*pi).*sin(2*pi*(t - t(1) + T/2));
        
        % check if the generated trajectory satisfies the requirements
        %  1) check if the joint velocities are within the limits
        if min(dq(j,:,tr)) >= dq_min_(j) && max(dq(j,:,tr)) <= dq_max_(j) 
            % now check if the trajectory is within the joint position limits
            if min(q(j,:,tr)) >= q_min_(j) && max(q(j,:,tr)) <= q_max_(j) 
                % now check if the trajectory is within the joint acceleration limits
                if min(ddq(j,:,tr)) >= ddq_min(j) && max(ddq(j,:,tr)) <= ddq_max(j) 
                    next = true; % if all the requirements are satisfied, go to the next joint
                end
            else
                % check if a shift is possible, otherwise... 
                if abs(max(q(j,:,tr)) - min(q(j,:,tr))) < (q_max_(j) - q_min_(j))
                    % now check if the trajectory is within the joint acceleration limits
                    if min(ddq(j,:,tr)) >= ddq_min(j) && max(ddq(j,:,tr)) <= ddq_max(j) 
                        q(j,:,tr) = q(j,:,tr) -(max(q(j,:,tr)) + min(q(j,:,tr)))/2 + (q_max_(j) + q_min_(j))/2;
                        next = true; % if all the requirements are satisfied, go to the next joint
                    end
                else % ... change parametes and try again
                    if trials == 50  % modify parameters and retry
                        a_a = a_a - 0.001; 
                        b_a = -a_a;
                        a_b = a_b - 0.002; 
                        b_b = -a_b;
                        trials = 0;
                    end
                    trials = trials + 1;
                end

            end
        else

            if trials == 50  % modify parameters and retry
                a_a = a_a - 0.002; 
                b_a = -a_a;
                a_b = a_b - 0.001; 
                b_b = -a_b;
                trials = 0;
            end
            trials = trials + 1;
        end
        
        % only in the successful case we will go further to the next joint    
        if next
            j = j + 1;
        end        
    end % A trajectory was generated
    
    % check if, with the generated trajectory, the end-effector doesn't 
    % collide with the table on which is mounted. Be aware that self
    % collision are not avoided and a further check should be performed.
    collide = false;
    for k=1:size(tt,2)
        [p,~] = DirectKinematic(DH,[q(:,k,tr); 0],Convention);
        if all(abs(p) < 0.3*ones(3,1)) || all(p(1) > -0.15 & abs(p(2))< 0.5 & p(3) < 0.3 )
            collide = true;
            break;            
        end
    end


%% plot and save generated trajectories 
%  Verify positions, velocities and accelerations are within the limits
    if not(collide)

        %figure(4*(tr-1) +1 )
        figure(1)
        annotation('textbox', [0 0.9 1 0.1], 'String', 'Check Joint positions respect the limits', 'EdgeColor', 'none','HorizontalAlignment', 'center')
        for j=1:N
            subplot(4,2,j)
            plot(tt,q(j,:,tr),'g-',[tt(1) , tt(end)],[q_min(j),q_min(j)],'r-',[tt(1) , tt(end)],[q_max(j), q_max(j)],'r-')
            grid on
            ylabel(strcat('q_{',num2str(j),'}'))
        end

        %figure(4*(tr-1) +2)
        figure(2)
        annotation('textbox', [0 0.9 1 0.1], 'String', 'Check Joint velocities respect the limits', 'EdgeColor', 'none','HorizontalAlignment', 'center')
        for j=1:N
            subplot(4,2,j)
            plot(tt,dq(j,:,tr),[tt(1) , tt(end)],[dq_min(j),dq_min(j)],'r-',[tt(1) , tt(end)],[dq_max(j), dq_max(j)],'r-')
            xlim([0 tt(end)])
            grid on
            ylabel(strcat('dq_{',num2str(j),'}'))
        end
        pause(1)

        %figure(4*(tr-1) +3)
        figure(3)
        annotation('textbox', [0 0.9 1 0.1], 'String', 'Check Joint accelerations respect the limits', 'EdgeColor', 'none','HorizontalAlignment', 'center')
        for j=1:N
            subplot(4,2,j)
            plot(tt,ddq(j,:,tr),'b-',[tt(1) , tt(end)],[ddq_min(j),ddq_min(j)],'r-',[tt(1) , tt(end)],[ddq_max(j), ddq_max(j)],'r-')
            xlim([0 tt(end)])
            grid on
            ylabel(strcat('ddq_{',num2str(j),'}'))
        end

        % save trajectories in file
        disp('Check the trajectory plots')
        prompt = 'Do you want to keep and save this trajectory? [y/n]: ';
        str = input(prompt,'s');
        if strcmp('y',str)

            base = '../data/Exciting_Traj/Trajectory_';
            path = strcat(base,num2str(tr),'/');
            if ~exist(path, 'dir')
                mkdir(path)
            end
            fileName = strcat(path,'gen_time.txt');
            fileTimeID = fopen(fileName,'w+');
            fileName = strcat(path,'gen_dq.txt');
            fileVelocityID = fopen(fileName,'w+');
            fileName = strcat(path,'gen_q0.txt');
            fileInitPosID = fopen(fileName,'w+');
            fileName = strcat(path,'gen_params.txt');
            fileParamasID = fopen(fileName,'w+');
            
            % Since there is no guarantee that self-collisions will be
            % avoided, we use the stored trajectory to feed a V-REP scene with
            % the robot in order to check if self collisions happens, if
            % not, the trajectory can be safely used to excite the real
            % robot.
            
            dq_vrep = reshape(dq(:,:,tr), 7*size(dq(:,:,tr),2),1);
            % save the time vector
            for i = 1 : length(tt)
                fprintf(fileTimeID,'%1.4f\n',tt(i));
            end
            % save the joint velocities
            fileTimeID = fclose(fileTimeID);
            for i = 1 : size(dq_vrep,1)
                fprintf(fileVelocityID,'%1.6f\n',dq_vrep(i));
            end
            fileVelocityID = fclose(fileVelocityID);
            % save the initial joint position
            for i = 1 : 7
                fprintf(fileInitPosID,'%1.4f\n',q(i,1,tr));
            end
            fileInitPosID = fclose(fileInitPosID);
            % save the harmonic amplitudes that characterize the trajectory 
            params = [a_k_l(:,:,tr);b_k_l(:,:,tr)];
            for i=1:14
                fprintf(fileParamasID,'%1.4f\t%1.4f\t%1.4f\t%1.4f\t%1.4f\n',params(i,:));
            end
            fileParamasID = fclose(fileParamasID);

            tr = tr + 1;
        end
        clc
    end

end

