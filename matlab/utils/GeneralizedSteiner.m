function [J] = GeneralizedSteiner(T,I,m)
%

R = T(1:3,1:3);
r = T(1:3,4);

J = R*I*R' + m*(r'*r*eye(3)-r*r');
end

