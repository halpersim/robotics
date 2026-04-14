%
% Fits (by translation and rotation) data points to a set of 
% line segments, i.e. applying the Cox's algorithm
% function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
% 
function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
    % Init variables
    ddx = 0; ddy = 0; dda = 0;
    Rx = POSE(1); Ry = POSE(2); Ra = POSE(3); 
    sALFA = SensorPose(1); sBETA = SensorPose(2); sGAMMA = SensorPose(3);
    max_iterations = 10; % <------YOU NEED TO CHANGE THIS NUMBER
    no_update = 0;
    
    % Step 0 - Normal vectors (length = 1) to the line segments
    % -> Add your code here
    % LINEMODEL Format X1 Y1 X2 Y2
    % line vector = [X2-X1, Y2-Y1]
    % 90° rotation = [-(Y2-Y1), X2-X1]
    wall_unit_vectors = [-(LINEMODEL(:,4) - LINEMODEL(:,2)), LINEMODEL(:,3) - LINEMODEL(:,1)];
    wall_unit_vectors = wall_unit_vectors ./ vecnorm(wall_unit_vectors, 2,2);
    dist_unit_to_wall = dot(wall_unit_vectors, [LINEMODEL(:,1), LINEMODEL(:, 2)], 2);

    range = 1:numel(DIS);
    
    % The Loop - REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations
        % Step 1 Translate and rotate data points
            % 1.1) Relative measurements => Sensor co-ordinates
            x = DIS(range) .* cos(ANG(range));
            y = DIS(range) .* sin(ANG(range));

            % 1.2) Sensor co-ordinates => Robot co-ordinates
            R = [cos(sGAMMA), -sin(sGAMMA), sALFA;...
                 sin(sGAMMA), cos(sGAMMA), sBETA;...
                 0, 0, 1];
            Xs = R*[x; y; ones(1, numel(x))];

            % 1.3) Robot co-ordinates => World co-ordinates
            R = [cos(Ra + dda), -sin(Ra + dda), Rx + ddx;...
                sin(Ra + dda), cos(Ra + dda), Ry + ddy;...
                0, 0, 1];
            meas_world = R*Xs;
      

        rep_walls = repmat(wall_unit_vectors, size(meas_world,2),1);
        rep_world = repelem(meas_world(1:2, :)', size(wall_unit_vectors, 1), 1);
        rep_meas = repmat(dist_unit_to_wall,size(meas_world, 2), 1);
        
        dot_values = dot(rep_walls, rep_world, 2);

        % Step 2 Find targets for data points
        % for each measurement point calculate the distance to each modelled line
        %dist_meas_to_wall_orig = repelem(dist_unit_to_wall,size(meas_world, 2), 1) - dot(repmat(wall_unit_vectors, size(meas_world,2),1), repelem(meas_world(1:2, :)', size(wall_unit_vectors, 1), 1), 2);
        %dist_meas_to_wall_orig = repmat(dist_unit_to_wall,size(meas_world, 2), 1) - dot(repmat(wall_unit_vectors, size(meas_world,2),1), repelem(meas_world(1:2, :)', size(wall_unit_vectors, 1), 1), 2);
        dist_meas_to_wall_orig = rep_meas - dot_values;
        % reshape the distances such that each column correspons to one datapoint
        dist_meas_to_wall = reshape(dist_meas_to_wall_orig, size(wall_unit_vectors, 1), size(meas_world,2));
        [closest_distance_meas, wall_idx] = min(abs(dist_meas_to_wall));

        colororder("gem12");
        col = colororder;
        figure;
        hold on;
        for j = 1:size(LINEMODEL, 1)
            wall_color = col(mod(j-1, 12)+1, :);

            plot([LINEMODEL(j, 1), LINEMODEL(j, 3)], [LINEMODEL(j, 2), LINEMODEL(j, 4)], "Color", wall_color);
            plot(meas_world(1, wall_idx==j), meas_world(2, wall_idx==j), "o", "Color", wall_color);
            text([LINEMODEL(j, 1)], [LINEMODEL(j, 2)], "" + j);
        end


        % discard outliers
        threshold = 150;
        valid_measurements = abs(closest_distance_meas) < threshold;
        wall_idx(~valid_measurements) = [];


        % Step 3 Set up linear equation system, find b = (dx,dy,da)' from the LS
        %-> Add your code here
        
        A = [wall_unit_vectors(wall_idx, :), dot(wall_unit_vectors(wall_idx, :), ([0,-1;1,0] * (meas_world(1:2, valid_measurements) - [Rx+ddx;Ry+ddy]))', 2)];
        y = dist_unit_to_wall(wall_idx) - dot(wall_unit_vectors(wall_idx, :), meas_world(1:2, valid_measurements)', 2); 
        b = pinv(A)*y;
        % for demonstation, i.e. return the same pose as sent in.

        n = max(size(A));
        est_var =(y-A*b)'*(y-A*b)/(n-4);
        % covariance matrix
        C = inv(A'*A) * est_var;

        % Step 4 Add latest contribution to the overall congruence 
        
        ddx = ddx + b(1);
        ddy = ddy + b(2);
        dda = dda + b(3);

        % Step 5  Check if the process has converged
        if (sqrt(b(1)^2 + b(2)^2) < 5) && (abs(b(3)) < 0.1*pi/180)
            break;
        end
    end
end

    