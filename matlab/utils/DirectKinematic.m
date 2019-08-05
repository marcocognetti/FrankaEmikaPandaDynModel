%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DirectKinematic(DH,q,Convention) computes the direct kinematics of a    %
% robot represented through its D-H table in a certain given              %
% configuration q                                                         %
% The format of the D-H table have to be as in the following example:     %
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


function [p,R] = DirectKinematic(DH,q,Convention)
    chain = FramesChainFromDH(DH,q,Convention);
    p = chain(1:3,4,end);
    R = chain(1:3,1:3,end);
end

