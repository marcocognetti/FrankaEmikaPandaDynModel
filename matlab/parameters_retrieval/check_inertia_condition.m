% author: Claudio Gaz, Marco Cognetti
% date: August 2, 2019
% 
% -------------------------------------------------
% Parameters Retrieval Algorithm
% -------------------------------------------------
% C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
% Identification of the Franka Emika Panda Robot With Retrieval of Feasible
% Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
%
% the following code has been tested on Matlab 2018b

% check_inertia_condition evaluates the external penalty due to the
% possible violation of the inertia tensor triangular inequality. See eq.
% (16) of the manuscript
function loss = check_inertia_condition(I,loss,penalty)

cond = trace(I)/2 - max(eig(I));
if cond < 0
    loss = loss - penalty*cond;
end    