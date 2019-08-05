%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This scrip transform the retrieved dynamic parameters of the Franka   %
%   Emika Panda Robot into a format compatible with the V-REP interface.  %
%   It also imports the robot's STL model together with the               %
%   Barycentral reference frame of each link, represented with a sphere,  %
%   in order to visualize the location of the CoM within the links convex %
%   hull. The input values are those reported in tables VIII and IX of    %
%   the Supplementary material of  our paper:                             %
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,         %
%'Dynamic Identification of the Franka Emika Panda Robot With Retrieval of%
% Feasible Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019. %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

addpath ../utils ../data/FrankaSTLModel

[DH, Convention] = Load_Franka_DH();
q = [ 0; 0; 0; 0; 0; 0; 0; 0];
chain = FramesChainFromDH(DH,q,Convention);

%% Load the STL model. The loaded meshes are the convex hull of the links
LoadFrankaSTLModel

PlotChain(chain);
axis vis3d equal
hold on
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-0.3 0.3]) 
ylim([-0.3 0.3])
zlim([0 1.2])

%%                     Retrived Dynamic Parameters
Il=[7.0337e-01, -1.3900e-04, 6.7720e-03, 7.0661e-01, 1.9169e-02, 9.1170e-03;
    7.9620e-03, -3.9250e-03, 1.0254e-02, 2.8110e-02, 7.0400e-04, 2.5995e-02;
    3.7242e-02, -4.7610e-03, -1.1396e-02, 3.6155e-02, -1.2805e-02, 1.0830e-02;
    2.5853e-02, 7.7960e-03, -1.3320e-03, 1.9552e-02, 8.6410e-03, 2.8323e-02;
    3.5549e-02, -2.1170e-03, -4.0370e-03, 2.9474e-02, 2.2900e-04, 8.6270e-03;
    1.9640e-03, 1.0900e-04, -1.1580e-03, 4.3540e-03, 3.4100e-04, 5.4330e-03;
    1.2516e-02, -4.2800e-04, -1.1960e-03, 1.0027e-02, -7.4100e-04, 4.8150e-03];

m =[4.970684
    0.646926
    3.228604
    3.587895
    1.225946
    1.666555
    7.35522e-01]; 

CoM_l =[ 3.875e-03,2.081e-03,0;
      -3.141e-03,-2.872e-02,3.495e-03;
       2.7518e-02,3.9252e-02,-6.6502e-02;
      -5.317e-02,1.04419e-01,2.7454e-02;
      -1.1953e-02,4.1065e-02,-3.8437e-02;
       6.0149e-02,-1.4117e-02,-1.0517e-02;
       1.0517e-02,-4.252e-03,6.1597e-02]';
   
%% Transform and display the values to be used in V-REP
[X,Y,Z] = sphere(10);
k=0.015;

w_CoM = zeros(size(CoM_l));
J = zeros(3,3,7);
a =['x';'y';'z'];

for i=1:7
    aux = chain(:,:,i)*[CoM_l(:,i);1];
    w_CoM(:,i) = aux(1:3);
    surf(w_CoM(1,i)+k*X,w_CoM(2,i)+k*Y,w_CoM(3,i)+k*Z)
    hold on
    msg = strcat('Franka_link ',num2str(i+1));
    disp(msg)
    msg = strcat('mass =  ',num2str(m(i)),' [Kg]');
    disp(msg)
    disp('Inertia matrix [m^2]= ')
    T = [chain(1:3,1:3,i),CoM_l(:,i) ;0 0 0 1];
    % express the Barycentral Inertia tensor in link frame (are parallels)
    % and orient it as the base (world) frame and depurate it from its mass
    J(:,:,i) = GeneralizedSteiner(T,vec2sm(Il(i,:),3),m(i));
    msg = num2str(J(:,:,i)/m(i));
    T = [chain(1:3,1:3,i),w_CoM(:,i);0 0 0 1];
    PlotFrame(T,strcat('{I_',num2str(i+1),'}'));
    disp(msg)
    disp('CoM in World Coord. = ')
    msg = strcat(a,' = ',num2str(w_CoM(:,i)),' [m]');
    disp(msg)
    disp('-------------------------------------------')
end
