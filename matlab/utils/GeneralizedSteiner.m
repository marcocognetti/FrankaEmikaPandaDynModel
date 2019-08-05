%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GeneralizedSteiner(T,I,m) referes the inertia tensor I in the frame     %
% given by the homogeneous transformation matrix T. m is the link mass.   %
% The function computes the Steiner's Parallel axis theorem and after     %
% rotates it to the desired given orientation.                            %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [J] = GeneralizedSteiner(T,I,m)

  R = T(1:3,1:3);
  r = T(1:3,4);

  J = I + m*(r'*r*eye(3)-r*r');
  J = R*J*R'; 
end

