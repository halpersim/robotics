close all;

fileID = fopen('debug_data/rot_1.txt','r');

formatSpec = '%d';
A = fscanf(fileID,formatSpec);
fclose(fileID);

A = reshape(A, [], 2);

ang = -deg2rad(0:2:358)';
dist = A(:, 1);

figure;
plot(cos(ang) .* dist, sin(ang) .* dist, "go");

make_arena;
ROBOT_POSE = [500, 500, 0];
[ddx,ddy,dda,C] = Cox_LineFit(ang', dist', ROBOT_POSE, LINEMODEL, [alfa beta gamma]);

