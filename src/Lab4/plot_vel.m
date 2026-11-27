% We're intentionally leaving this one a bit open ended to prepare you 
% for next week.
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
recordings = zeros(376, 11);
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
        targVel = trajGen.eval_traj_vel(coeffs,toc);

        % Pass your targets x, y, z, alpha into IK to get joint values
        joints = robot.ik3001(transpose(points));
        robot.interpolate_jp(joints, 0.05)

        % Record the data
        curVel = robot.measure_js(false, true);
        curJac = robot.dk3001(joints, curVel(2,:).');
        curJac2 = curJac.';
        recordings(j, 1:4) = targVel;
        recordings(j,5:10) = curJac2;
        recordings(j, 11) = prev_time+toc;
        j = j+1;
    end
    prev_time = prev_time+toc;
end

% Plot the trajectory of your robot in x-y-z space using scatter3
hold on
plot(recordings(:,11), recordings(:,1));
plot(recordings(:,11), recordings(:,2));
plot(recordings(:,11), recordings(:,3));
plot(recordings(:,11), recordings(:,5));
plot(recordings(:,11), recordings(:,6));
plot(recordings(:,11), recordings(:,7));
plot(recordings(:,11), recordings(:,8));
plot(recordings(:,11), recordings(:,9));
plot(recordings(:,11), recordings(:,10));
hold off
xlabel('Time');
ylabel('X,Y,Z Velocities');
legend ("Expected X Vel", "Expected Y Vel", "Expected z Vel", ...
        "Actual X Vel", "Actual Y Vel", "Actual Z Vel", ...
        "Actual Roll Vel", "Actual Pitch Vel", "Actual Yaw Vel");
