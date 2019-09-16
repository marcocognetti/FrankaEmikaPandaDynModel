% author: Claudio Gaz, Marco Cognetti, Alexander Oliva
% date: September 3, 2019
% 
% -------------------------------------------------
% Panda Dynamic Model v. 1.0
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
%
% the following code has been tested on Matlab 2018b

% this is a test file, exploiting the dynamic model functions

clear all
close all
clc

t_in = 0; % [s]
t_fin = 5; % [s]
delta_t = 0.001; % [s]

num_of_joints = 7;

t = t_in:delta_t:t_fin;

Q = zeros(num_of_joints,length(t));
dQ = zeros(num_of_joints,length(t));
ddQ = zeros(num_of_joints,length(t));
TAU = zeros(num_of_joints,length(t));

for j=1:num_of_joints
    Q(j,:) = sin(t);
    dQdt(j,:) = cos(t);
    ddQdt(j,:) = -sin(t);
end

for i=1:length(t)
    q = Q(:,i);
    dq = dQdt(:,i);
    ddq = ddQdt(:,i);
    
    g = get_GravityVector(q);
    c = get_CoriolisVector(q,dq);
    M = get_MassMatrix(q);
    tauf = get_FrictionTorque(dq);
    
    TAU(:,i) = M*ddq + c + g + tauf;
    
    % equivalently, you could use (decomment) the following two lines:
%     Cmat = get_CoriolisMatrix(q,dq);
%     TAU(:,i) = M*ddq + Cmat*dq + g + tauf;
end

figure
for j=1:num_of_joints
    subplot(4,2,j);
    plot(t,TAU(j,:))
    xlabel('time [s]');
    ylabeltext = sprintf('_%i [Nm]',j);
    ylabel(['\tau' ylabeltext]);
    grid;
end
