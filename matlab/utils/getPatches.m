%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% getPatches( ) loads an aproximation of the robot's environment in our   %
% lab (base and side tables)                                              %
%                                                                         %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [patches] = getPatches()

support_table_vertices = [ 
   0.135   0.135   0.0 ;  % 1
   0.135  -0.135   0.0 ;  % 2
  -0.135  -0.135   0.0 ;  % 3
  -0.135   0.135   0.0 ;  % 4
   0.135   0.135  -0.81;  % 5   
   0.135  -0.135  -0.81;  % 6
  -0.135  -0.135  -0.81;  % 7
  -0.135   0.135  -0.81   % 8
  ];
support_table_faces = [
    1 2 3 4;
    1 4 8 5;
    4 3 7 8;
    2 3 7 6;
    5 6 7 8;
    1 2 6 5
    ];
work_table_vertices = [
   0.135   0.4  0.09 ;  % 9
   0.135  -0.4  0.09 ;  % 10
   0.135  -0.4 -0.81;  % 11
   0.135   0.4 -0.81;  % 12
   0.935   0.4  0.09 ;  % 13
   0.935  -0.4  0.09 ;  % 14
   0.935  -0.4 -0.81;  % 15
   0.935   0.4 -0.81   % 16
   ];
work_table_faces = [
    9 10 11 12;
    9 10 14 13;
    9 12 16 13;
    10 11 15 14;
    13 14 15 16;
    11 12 16 15
    ];
side_table_vertices = [
   0.975  -0.4  -0.05 ;  % 17
   0.975  -1.2  -0.05 ;  % 18
  -0.225  -1.2  -0.05 ;  % 19
  -0.225  -0.4  -0.05 ;  % 20
   0.975  -0.4  -0.81;  % 21
   0.975  -1.2  -0.81;  % 22
  -0.225  -1.2  -0.81;  % 23
  -0.225  -0.4  -0.81;  % 24
  ];
side_table_faces =[
    17 18 19 20;
    17 20 24 21;
    17 18 22 21;
    18 19 23 22;
    19 20 24 23;
    21 22 23 24
    ];
env.vertices = [
    support_table_vertices;
    work_table_vertices;
    side_table_vertices];    
env.faces = [ 
    support_table_faces;
    work_table_faces   
    side_table_faces];

s = 0.1;

safety_env = env;

for i=1:size(safety_env.vertices,1)
    for j=1:size(safety_env.vertices,2)
        if (safety_env.vertices(i,j) >= 0 )
            safety_env.vertices(i,j) = safety_env.vertices(i,j) + s;
        elseif (safety_env.vertices(i,j) < 0 && safety_env.vertices(i,j)~= -0.81)
            safety_env.vertices(i,j) = safety_env.vertices(i,j) - s;
%         else
%             text = sprintf('vertex in position (%d,%d) is in the origin, manage it manually', i,j);
%             display(text)
        end
    end
end

for i=9:12
    safety_env.vertices(i,1) = safety_env.vertices(i,1) - 2*s;
end
for i=17:20
    safety_env.vertices(i,3) = safety_env.vertices(i,3) + 2*s;
end
safety_env.vertices(17,2) = safety_env.vertices(17,2) + 2*s;
safety_env.vertices(20,2) = safety_env.vertices(20,2) + 2*s;
safety_env.vertices(21,2) = safety_env.vertices(21,2) + 2*s;
safety_env.vertices(24,2) = safety_env.vertices(24,2) + 2*s;

patches(1) = env; 
patches(2) = safety_env;


end