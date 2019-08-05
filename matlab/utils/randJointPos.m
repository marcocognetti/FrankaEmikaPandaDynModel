%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% randJointPos(q_min,q_max,n) generates a random joint configuration      %
% within the joint limits (q_min <= q <= q_max)                           %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [r] = randJointPos(q_min,q_max,n)

    r = (q_max(1:n)'-q_min(1:n)').*rand(n,1) + q_min(1:n)';
end

