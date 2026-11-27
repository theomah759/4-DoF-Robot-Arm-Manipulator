clc
% Create a robot object
robot = Robot();
interpolate_time = 3; % Defines travel time
homePos = zeros(1,4); % Defines home position
samples = 1000; % Number of Overall Samples that can be taken
m = 0; % Actual Samples Taken - Variable Size

storePos = zeros(samples,4); % Creates a matrix to store joint positions
timeStamp = zeros(samples,1); % Creates a matrix to store timestamps

x = zeros(1, 4); % Creates a matrix to store EE x position
y = zeros(1, 4); % Creates a matrix to store EE y position
z = zeros(1, 4); % Creates a matrix to store EE z position
theta2 = zeros(1, 4); % Creates a matrix to store joint 2's position
theta3 = zeros(1, 4); % Creates a matrix to store joint 3's position
theta4 = zeros(1, 4); % Creates a matrix to store joint 4's position

robot.interpolate_jp(homePos, interpolate_time); % Moves all joints to zero positions
pause(4); % Ensures all joints are able to reach zero position before setting target position


% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!


% Move your robot to the first vertex
pos = [0,   45, -45, -35; % Defines target joint positions
       0,  -20, -70,  90;
       0,    0,   0,   0];
 
% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
for i = 1:size(pos, 1)
    % See lab_demo.m from lab 1 for inspiration on how to move your robot
    % and collect data
    robot.interpolate_jp(pos(i,:),interpolate_time); % Moves arm to next set of joint positions

    tic;  % Start timer
    while toc < interpolate_time
        m = m + 1; % Records number of actual samples taken
        % Reads current joint positions (not velocities) and assigns matrix 
        % of values to the output variable
        output = robot.measure_js(true, false); % Records the current joint positions at each timestamp
    
        % Stores the positions in a matrix
        storePos(m, :) = output(1, :); % Store the current joint positions in a new row for every entry
    end
end

jointPos = storePos.'; %Transpose of storePos to get joint positions

for i = 1:m
    jointSpaceConfig = storePos(i, :); % Stores the positions for the joints at each timestamp
    taskspace = robot.fk3001(jointSpaceConfig); % Converts the joint positions to EE positions
    
    theta2(i) = jointPos(2,i); % Defines theta2 over time
    theta3(i) = jointPos(3,i); % Defines theta3 over time
    theta4(i) = jointPos(4,i); % Defines theta4 over time

    x(i) = taskspace(1,4);  % X position
    y(i) = taskspace(2,4);  % Y position
    z(i) = taskspace(3,4);  % Z position
end

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

% % Plot the trajectory of your robot in x-y-z space using scatter3
% % https://www.mathworks.com/help/matlab/ref/scatter3.html
figure;
grid on;
scatter3(theta2, theta3, theta4)
xlabel('theta2')
ylabel('theta3')
zlabel('theta4')
title('Trajectory of the Robot in Joint Space')
axis equal

% Plot the trajectory of your robot in theta2-theta3-theta4 space using
% scatter3
figure;
grid on;
scatter3(x, y, z)
title('Trajectory of the Robot in Task Space')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
