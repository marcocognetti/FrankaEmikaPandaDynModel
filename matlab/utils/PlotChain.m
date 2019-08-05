%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PlotChain( Ch ) plots the given kinematic chain.                        %
%                                                                         %
% By default the base frame is placed at r = (0, 0, 0) and aligned with   %
% the world frame (R = eye(3))                                            %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function PlotChain( Ch )

PlotFrame(eye(4),0);
hold on
for i=1:size(Ch,3)
    if i==size(Ch,3)
        % I am assuming that the last Frame is the End-Effector
        PlotFrame(Ch(:,:,i),'EE');
    else
        PlotFrame(Ch(:,:,i),i);
    end
end

hold off
end