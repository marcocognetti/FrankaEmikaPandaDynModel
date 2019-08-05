%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This script generates a sorted sequence of robot configurations.      % 
%   The output of this scrip is used to feed the C++ code that will move  %
%   the Panda robot in such configurations and will record in files some  %
%   parameters that will allow the identification of the Mass matrix, the %
%   Coriolis vector and the Gravity vector of the robot using a Reverse   %
%   Engineering approach.                                                 %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;


%% set the number of robot configuraions you want to generate
num_of_good_positions = 10;  % we generate 1010 different configurations

%% load robot parameters
num_of_joints = 7;
[DH, Convention] = Load_Franka_DH();

% maximum joint ranges (deg): (q1..q7)
%       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];
q_min = q_min + deg2rad([0 0 0 10 0 5 0]);
%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];
q_max = q_max - deg2rad([0 0 0 10 0 5 0]);

GOOD_CONFS = zeros(num_of_good_positions,num_of_joints);
P = zeros(3,num_of_joints + 1);
sim_count = 0;
k=1;
out = getPatches();
env = out(1);
safety_env = out(2);
titl = '';


while sim_count < num_of_good_positions
   
    q =  [randJointPos(q_min,q_max,num_of_joints); 0];
    KinematicChain = FramesChainFromDH(DH,q,Convention);
    
    for i=1:num_of_joints+1
        P(:,i) = KinematicChain(1:3,4,i);
    end
    
    tri = delaunayn(safety_env.vertices);        % Generate delaunay triangulization
    tn = tsearchn(safety_env.vertices, tri, P'); % Determine which triangle point is within
    IsInside = ~isnan(tn);                       % Convert to logical vector
    
    
    if ~(IsInside)
        sim_count = sim_count +1;
        res_msg_to_disp = sprintf('%d good configuration found',sim_count);
        disp(res_msg_to_disp);
        GOOD_CONFS(sim_count,:) = q(1:7)';
        titl = 'good';
    else
        res_msg_to_disp = sprintf('Bad configuration...');
        disp(res_msg_to_disp);
        titl = 'bad';
    end
    
    % ******comment this block for speed-up the generation proccess *******
    % the plot shows the configuration of the robot frames and its 
    % environment (in our laboratory). Use this feature to be sure that a
    % configuration doesn't brigs the robot to collide with something in 
    % its sorroundings. Be aware that self collisions are not avoided!!
    
    figure(1)
    PlotChain(KinematicChain);
    title(titl);
    axis square equal;
    patch(safety_env,'EdgeColor','red','FaceColor','red','LineWidth',1)
    alpha(0.3);
    patch(env,'EdgeColor','green','FaceColor','blue','LineWidth',2)
    k=k+1;
    pause(1)
    %**********************************************************************
        
end

%% Sorting poses
% 1) find the configuration with the smallest joint positions
min = q_max*q_max';
for i=1:size(GOOD_CONFS)
    m = sum(GOOD_CONFS(i,:));
    if m < min
        min = m;
        idx_min = i;
    end
end
% 2) from the smallest position, find the nearest position and iterate from
% the founded position
sorted_confs = zeros(num_of_good_positions,num_of_joints);
sorted_confs(1,:) = GOOD_CONFS(idx_min,:);
GOOD_CONFS(idx_min,:) = [];
j=1;
idx_min=1;
while size(GOOD_CONFS,1) > 0
    min = q_max*q_max';
    for k=1:size(GOOD_CONFS,1)
        m = sorted_confs(j,:)-GOOD_CONFS(k,:);
        m = m*m';
        if m <= min
            min = m;
            idx_min = k;
        end
    end
    j=j+1;
    sorted_confs(j,:) = GOOD_CONFS(idx_min,:);
    GOOD_CONFS(idx_min,:) = [];
end

%% save the extracted configurations in a file.

dlmwrite('../data/good_confs/GOOD_CONFS.txt',sorted_confs,'delimiter','\t');

%% Plot the distribution of the extracted configurations.
% They have to span as much as possible the position range
figure(2)
hold on
grid on
plot(1,q_min(1),'r*',2,q_min(2),'r*',3,q_min(3),'r*',...
       4,q_min(4),'r*',5,q_min(5),'r*',6,q_min(6),'r*',...
       7,q_min(7),'r*')
plot(1,q_max(1),'r*',2,q_max(2),'r*',3,q_max(3),'r*',...
       4,q_max(4),'r*',5,q_max(5),'r*',6,q_max(6),'r*',...
       7,q_max(7),'r*')
for k=1:size(sorted_confs,1)
   plot(1,sorted_confs(k,1),'b*',2,sorted_confs(k,2),'b*',3,sorted_confs(k,3),'b*',...
       4,sorted_confs(k,4),'b*',5,sorted_confs(k,5),'b*',6,sorted_confs(k,6),'b*',...
       7,sorted_confs(k,7),'b*') 
   
end
title('Joint positions distribution within the admissible range')
xticks([1:7])
xticklabels({'joint 1' 'joint 2' 'joint 3' 'joint 4' 'joint 5' 'joint 6' 'joint 7'})
xlim([0 8])

