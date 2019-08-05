%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Loads the STL Model of each link of the Franka Emika Panda Robot.     %  
%   If the variable 'with_hand' is set to 'true', also the gripper will   %
%   be loaded and visualized. Link frames are also reported               %
%                                                                         %
%   Author: Alexander Oliva                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

file_link0 = '../data/FrankaSTLModel/link0.stl';
file_link1 = '../data/FrankaSTLModel/link1.stl';
file_link2 = '../data/FrankaSTLModel/link2.stl';
file_link3 = '../data/FrankaSTLModel/link3.stl';
file_link4 = '../data/FrankaSTLModel/link4.stl';
file_link5 = '../data/FrankaSTLModel/link5.stl';
file_link6 = '../data/FrankaSTLModel/link6.stl';
file_link7 = '../data/FrankaSTLModel/link7.stl';
file_hand = '../data/FrankaSTLModel/hand.stl';
file_finger = '../data/FrankaSTLModel/finger.stl';

with_hand = false;


[DH, Convention] = Load_Franka_DH();
q = zeros(8,1);
chain = FramesChainFromDH(DH,q,Convention);
PlotChain(chain);
axis vis3d equal
hold on
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-0.3 0.3]) 
ylim([-0.3 0.3])
zlim([0 1.2])

link0 = stlread(file_link0);
patch(link0,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link1 = stlread(file_link1);
link1.vertices = (link1.vertices' + chain(1:3,4,1))';
patch(link1,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);
 
link2 = stlread(file_link2);
Rx = eul2rotm(deg2rad([0 0 -90]),'ZYX');
link2.vertices = (Rx*link2.vertices' + chain(1:3,4,2))';
patch(link2,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link3 = stlread(file_link3);
link3.vertices = (link3.vertices' + chain(1:3,4,3))';
patch(link3,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link4 = stlread(file_link4);
Rx = eul2rotm(deg2rad([0 0 90]),'ZYX');
link4.vertices = (Rx*link4.vertices' + chain(1:3,4,4))';
patch(link4,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link5 = stlread(file_link5);
link5.vertices = (link5.vertices' + chain(1:3,4,5))';
patch(link5,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link6 = stlread(file_link6);
Rx = eul2rotm(deg2rad([0 0 90]),'ZYX');
link6.vertices = (Rx*link6.vertices' + chain(1:3,4,6))';
patch(link6,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

link7 = stlread(file_link7);
Rx = eul2rotm(deg2rad([0 0 180]),'ZYX');
link7.vertices = (Rx*link7.vertices' + chain(1:3,4,7))';
patch(link7,'FaceColor',       [0.8 0.8 1.0], ...
'EdgeColor',       'black',        ...
'FaceLighting',    'gouraud',     ...
'AmbientStrength', 0.15);

if with_hand
    
    hand_patch = stlread(file_hand);
    Rx = eul2rotm(deg2rad([0 0 180]),'ZYX');
    hand_patch.vertices = (Rx*hand_patch.vertices' + chain(1:3,4,8))';
    patch(hand_patch,'FaceColor',       [0.8 0.8 1.0], ...
    'EdgeColor',       'black',        ...
    'FaceLighting',    'gouraud',     ...
    'AmbientStrength', 0.15);

    finger_patch = stlread(file_finger);
    Ry = eul2rotm(deg2rad([0 180 0]),'ZYX');
    finger_patch.vertices = (Ry*finger_patch.vertices' + chain(1:3,4,8) + [0;0.03;-0.065])';
    patch(finger_patch,'FaceColor',       [0.8 0.8 1.0], ...
    'EdgeColor',       'black',        ...
    'FaceLighting',    'gouraud',     ...
    'AmbientStrength', 0.15);
    finger_patch = stlread(file_finger);
    yaw = deg2rad(180);
    Ry = eul2rotm(deg2rad([0 180 0]),'ZYX');
    Rz = eul2rotm(deg2rad([180 0 0]),'ZYX');
    finger_patch.vertices = (Rz*Ry*finger_patch.vertices' + chain(1:3,4,8) + [0;-0.03;-0.065])';
    patch(finger_patch,'FaceColor',       [0.8 0.8 1.0], ...
    'EdgeColor',       'black',        ...
    'FaceLighting',    'gouraud',     ...
    'AmbientStrength', 0.15);

end

alpha(0.4)
