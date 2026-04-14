clear;
close all;

% alfa = 660;
% beta = 0;
% gamma = -90*pi/180;

alfa = 0;
beta = 0;
gamma = 0;

ROBOT_POSE = [500; 500; 0];

ARENA_X = 2425;
ARENA_Y = 3630;
    
LINEMODEL = [0, 0, ARENA_X, 0; ... 			    % South Wall
	        ARENA_X, 0, ARENA_X, ARENA_Y; ...	% East Wall
	        ARENA_X, ARENA_Y, 0, ARENA_Y; ...	% North Wall
	        0, ARENA_Y, 0, 0]; 			    % West Wall

fileID = fopen('one_scan_out.txt','r');

formatSpec = '%d %d %d';
A = fscanf(fileID,formatSpec);
A = reshape(A, 3, [])';

angle = deg2rad(-A(:,2));
dist = A(:,3);
qual = A(:, 1);

angle(qual < 1) = [];
dist(qual < 1) = [];


[ddx,ddy,dda,C] = Cox_LineFit(angle', dist', ROBOT_POSE, LINEMODEL, [alfa beta gamma]);


figure;
hold on;
for k = 1:size(LINEMODEL, 1)
    plot(LINEMODEL(k, [1, 3]), LINEMODEL(k, [2, 4]), "b");
end
%plot(points_rotate(1, :), points_rotate(2, :), "ro");


%plot(pose_x, pose_y, "k.", "MarkerSize", 3);
%plot([pose_x, cos(angle) + pose_x], [pose_y, sin(angle) + pose_y], "k");

plot(ROBOT_POSE(1)-ddx, ROBOT_POSE(2)-ddy, "go", "MarkerSize", 3);
plot(ROBOT_POSE(1) + [-ddx, cos(-dda) - ddx], ROBOT_POSE(2) + [-ddy, sin(-dda) - ddy], "g");

%trans = [cos(angle), -sin(angle), pose_x; sin(angle), cos(angle), pose_y; 0, 0, 1];
inv_trans = [cos(dda), -sin(dda), ddx; sin(dda), cos(dda), ddy; 0, 0, 1];
