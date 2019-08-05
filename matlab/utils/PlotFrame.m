%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PlotFrame(wTa,n) draws a 3D orthogonal reference frame placed and       %
% oriented as indicated in the homogeneous matrix wTa w.r.t. the world    %
% frame. Parameter 'n' is used as subscript of the frame axis.            %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [q] = PlotFrame(wTa,n)


if nargin < 2
    n = -1;
end

pos = wTa(1:3,4)';
pos = repmat(pos,3,1);

q = quiver3(pos(1:3,1),pos(1:3,2),pos(1:3,3),wTa(1,1:3)',wTa(2,1:3)',wTa(3,1:3)');
asf = 0.1;
q.AutoScaleFactor = asf;

% shift the label of Y axis a bit in order to avoid labels overlap
wTa(1:3,2)=wTa(1:3,2)+0.1*ones(3,1);
if isnumeric(n)
    if n == -1
        n = '';
    else
        n=num2str(n);
    end
end
p=strcat('{',n,'}');
labels = [strcat('X_',p);strcat('Y_',p);strcat('Z_',p)];
text(pos(1:3,1)+asf*wTa(1,1:3)',pos(1:3,2)+asf*wTa(2,1:3)',pos(1:3,3)+asf*wTa(3,1:3)',labels);

cd_tail = uint8([blkdiag([255 255],[255 255],[255 255]);255*ones(1,6)]);
set(q.Tail,'ColorBinding','interpolated','ColorData',cd_tail,'LineWidth',3);
cd_head = uint8([blkdiag([255 255 255],[255 255 255],[255 255 255]);255*ones(1,9)]);
set(q.Head,'ColorBinding','interpolated','ColorData',cd_head,'LineWidth',3);

end
