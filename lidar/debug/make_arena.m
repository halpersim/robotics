
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
