clc 

robot = Robot();
%% Step 1: Define your symbolic DH table
% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variables and plain-ol numbers.

syms theta1 theta2 theta3 theta4

identityMatrix = eye(4);

dh_tab = [     0,        60.25,    0,     0;
             theta1,      36.076,    0,   -90;
           theta2-79.38,    0,   130.231,  0;
           theta3+79.38,    0,      124,   0;
             theta4,        0,     133.4,  0];

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix
% It's really that simple. MATLAB will leave everything unsolved
% Show this to an SA for SIGN-OFF #4

symFK_matrix = robot.dh2fk(dh_tab);


%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.

% Write the fk_3001 function in Robot.m to complete sign-off #5

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

FK = matlabFunction (symFK_matrix, 'Vars', [theta1, theta2, theta3, theta4], 'File', 'symFK_matrix');

zeros = ([0, 0, 0, 0]);

result = robot.fk3001(zeros);


