%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load_Franka_DH() Builds and returns the D-H table of the Franka Emika   %
% Panda Robot (Modified DH).                                              %
% The kinematic chain is built until the flange frame, for that           %
% reason it seems to have 8 joints but, the last one is kept fixed.       %
% The format of the D-H table follows the example:                        %
% D-H table format example:                                               %
%  joint type | d_i | theta_i | a_i  | alpha_i                            %
%       1     | d1  |    q1   |  0   |   pi/2                             %
%                                                                         %
% joint type: 1 'revolute', 2 'prismatic'                                 %
% Convention: 'classic' (default) | 'modified'                            %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [DH, Convention] = Load_Franka_DH()

    j = ones(8,1);
    d = [0.333 0 0.316 0 0.384 0 0 0.107]';
    theta = [0 0 0 0 0 0 0 0]';
    a = [0 0 0 0.0825 -0.0825 0 0.088 0]';
    alphas = [0 -pi/2 pi/2 pi/2 -pi/2 pi/2 pi/2 0 ]';
    Convention = 'modified';
    DH = [j, d, theta, a, alphas];

end