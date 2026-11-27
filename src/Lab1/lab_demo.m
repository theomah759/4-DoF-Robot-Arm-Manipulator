%% USE THIS FILE TO GET SIGN OFF 3
% Set Up and Variables
robot = Robot();  % Create a robot object to use
interpolate_time = 5; % Interpolate time of 5 seconds
reset_pos = zeros(1,4);
samples = 1000; % Number of Overall Samples that can be taken
m = 0; % Actual Samples Taken - Variable Size


%% Send the robot to its 0 position
% We are using interpolate_jp instead of servo_jp due to the concern of how
% fast the arm moves with servo_jp
robot.interpolate_jp(reset_pos, 3); % Moves all joints to zero positions
pause(3); % Ensures all joints are able to reach zero position before setting target position


%% Send the robot to an arbitrary position
pos1 = [-45, -20, 30,-30]; % Target position

%% Initialize an array full of 0s to store the positions
storePos = zeros(samples,4); % Makes an empty matrix to store positions in with size of "samples" x 4


%% Initialize another array for the timestamps
timeStamp = zeros(samples,1); % Makes an empty matrix to store times in with size of "samples" x 1


%% Call a function to move the robot
robot.interpolate_jp(pos1,interpolate_time);


%% Collect data as the robot moves
tic;  % Start timer
while toc < interpolate_time
    m = m + 1; % Records number of actual samples taken
    % Reads current joint positions (not velocities) and assigns matrix of
    % values to the output variable
    output = robot.measure_js(true, false);

    % Stores the positions and timesetamps in a matrix
    storePos(m, :) = output(1, :); % Store the current joint positions in a new row for every entry
    timeStamp(m, :) = toc; % Store the current time stamp in a new row for every entry
end
    
    % For Purposes of Reporting Clean Code for Graphing 
    Y1 = storePos(1:m,1);
    Y2 = storePos(1:m,2);
    Y3 = storePos(1:m,3);
    Y4 = storePos(1:m,4);
    X = timeStamp(1:m);
    X = X.*1000;

%% Make your figure
% Plot for all Joints by Degrees and Time
subplot(2,2,1)
plot(X,Y1)

xlim([0 5000])
title('Base Joint Movement in Degrees over Time (Joint 1)')
xlabel('Travel Time (milliseconds)') 
ylabel('Joint Position (degrees)')

subplot(2,2,2)
plot(X,Y2)
xlim([0 5000])
title('Arm Joint Movement in Degrees over Time (Joint 2)')
xlabel('Travel Time (milliseconds)') 
ylabel('Joint Position (degrees)')

subplot(2,2,3)
plot(X,Y3)
xlim([0 5000])
title('Elbow Joint Movement in Degrees over Time (Joint 3)')
xlabel('Travel Time (milliseconds)') 
ylabel('Joint Positions (degrees)')

subplot(2,2,4)
plot(X,Y4)
xlim([0 5000])
title('Wrist Joint Movement in Degrees over Time (Joint 4)')
xlabel('Travel Time (milliseconds)') 
ylabel('Joint Position (degrees)')


%% Calculate time step statistics
% Calculation of Additional Statistics
stepStats = diff(X);
timeMean = mean(stepStats);
timeMedian = median(stepStats);
timeMax = max(stepStats);
timeMin = min(stepStats);
