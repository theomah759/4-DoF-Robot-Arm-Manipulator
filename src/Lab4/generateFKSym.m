clc

robot = Robot();
%% Step 1: Define your symbolic DH table
% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variables and plain-ol numbers.

syms theta1 theta2 theta3 theta4

identityMatrix = eye(4);

dh_tab = [     0,         60.25,     0,     0;
             theta1,      36.076,    0,    -90;
           theta2-79.38,    0,    130.231,  0;
           theta3+79.38,    0,      124,    0;
             theta4,        0,     133.4,   0];

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

% zeros = ([0, 0, 0, 0]);

result = robot.fk3001([0, 0, 0, 0]);

%% LAB 4

% posJacobian = diff(result(3:4));
posJacobian = jacobian([symFK_matrix(1:3,4)], [theta1; theta2; theta3; theta4]);

% for each frame make the ht matrix 0 to frame
% take the z position of each matrix (1:3,3)
% add to angJacobian andJacobian(:,frame) = zPos
% add the angJacobian to the bottom of posJacobian

angJacobian = sym(zeros(3,4));
jacMatrix = sym(zeros(6,4));

t01 = robot.dh2mat(dh_tab(1,:));
t02 = robot.dh2fk(dh_tab(1:2,:))*t01;
t03 = robot.dh2fk(dh_tab(1:3,:))*t01;
t04 = robot.dh2fk(dh_tab(1:4,:))*t01;

angJacobian = [t01(1:3,3), t02(1:3,3), t03(1:3,3), t04(1:3,3)];

% for i = 1:4
%     ht = robot.dh2fk(dh_tab(1:i,:)); 
%     angJacobian(:,i) = ht(1:3, 3); 
%     % angJacoiban(:, i) = zPos;
%     jacMatrix(:,i) = [posJacobian(:,i); angJacobian(:,i)];
% end

% fkmat = eye(3,4);
% for i = 1:4 % iterate through all joints in the dh table
%     angJacobian(:,i) = fkmat(1:3,3);
%     fkmat = fkmat*robot.dh2mat(dh_tab(i,:));
% end

jacMatrix = [posJacobian; angJacobian];
disp(jacMatrix);

DK = matlabFunction (jacMatrix, 'Vars', [theta1, theta2, theta3, theta4], 'File', 'jacMatrix');
