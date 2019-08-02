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

% read_bounds reads a csv file containing upper and lower bounds for the
% dynamic parameters
function [LB,UB] = read_bounds(filename)

boundstable = readtable(filename);
bounds = table2array(boundstable(:,2:3));
    
LB = bounds(:,1);
UB = bounds(:,2);

end