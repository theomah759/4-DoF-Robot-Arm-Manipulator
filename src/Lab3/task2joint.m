% Create a robot object
robot = Robot();
trajGen = TrajGenerator();
travel_time = 2; 

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot
pos1 = [170, 0, 100, 0];
pos2 = [300, 0, 100, 0];
pos3 = [200, 0, 300, 0];
pos4 = pos1;

pos = [pos1; 
       pos2; 
       pos3;
       pos4];

t0 = 0;
tf = travel_time;
v0 = 0;
vf = 0;

% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

leg1_coeffs = zeros(4,4);
leg2_coeffs = zeros(4,4);
leg3_coeffs = zeros(4,4);

for j = 1:4   % x, y, z, alpha
    leg1_coeffs(j,:) = trajGen.cubic_traj(t0, tf, pos(1,j), pos(2,j), v0, vf);
    leg2_coeffs(j,:) = trajGen.cubic_traj(t0, tf, pos(2,j), pos(3,j), v0, vf);
    leg3_coeffs(j,:) = trajGen.cubic_traj(t0, tf, pos(3,j), pos(4,j), v0, vf);
end

% Move your robot to the first vertex
robot.interpolate_jp(robot.ik3001(pos(1,:)), travel_time);
pause(travel_time)

% Trajectory States
% Create a for loop that sends your robot to each vertex, recording the
% JOINT-SPACE position the entire time

j = 1; 
recordings = zeros(6000, 5);
prev_time = 0;

coeffs = zeros(4,4);

for i=1:3
    tic;

    if i == 1
        coeffs = leg1_coeffs;
    elseif i == 2
        coeffs = leg2_coeffs;
    else 
        coeffs = leg3_coeffs;
    end

    while toc < travel_time
        % Evaluate your trajectory using eval_traj
        points = trajGen.eval_traj(coeffs, toc);

        % Pass your targets x, y, z, alpha into IK to get joint values
        joints = robot.ik3001(transpose(points));
        robot.interpolate_jp(joints, 0.05)

        % Record the data
        tmp = measure_js(robot, true, false);
        recordings(j, 1:4) = tmp(1, :);
        recordings(j, 5) = + prev_time+toc;
        j = j+1;
    end
    prev_time = prev_time+toc;
end


% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

taskspace = zeros(j-1,4);
for i = 1:(j-1)
    V = robot.fk3001([recordings(i,1) recordings(i,2) recordings(i,3) recordings(i,4)]);
    taskspace(i, 1) = V(1,4);
    taskspace(i, 2) = V(2,4);
    taskspace(i, 3) = V(3,4);
    taskspace(i, 4) = recordings(i, 5);
end

% Plot the trajectory of your robot in x-y-z space using scatter3
subplot(1, 2, 1);
scatter3(taskspace(:,1), taskspace(:,2), taskspace(:,3), 50, taskspace(:, 4), "filled");
title('End Effector in  Task Space (mm)');
xlabel('Global X Position');
ylabel('Global Y Position');
zlabel('Global Z Position');


% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
subplot(1, 2, 2);
scatter3(recordings(:,2), recordings(:,3), recordings(:,4), 50, recordings(:, 5), "filled");
title('Trajectory of Robot in theta2-theta3-theta4 Space (deg)');
xlabel('theta2');
ylabel('theta3');
zlabel('theta4');
