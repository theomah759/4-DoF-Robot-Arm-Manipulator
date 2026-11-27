robot = Robot();
trajGen = TrajGenerator;
% Move your robot to a point where Y=0, X > 150 and Z > 150

% Generate a trajectory that will move your robot in a straight line from
% your current (X, Y, Z) to the point (X, -Y, Z)

%% Execute the trajectory
travelTime = 10;

homePos = zeros(1,4); % Defines home position

Y = []; 


det_j = [];

Y_target1 = [0, 281.4009, 300,0];
Y_target2 = [0, -281.4009, 300,0];
Y_targets = [Y_target1;
             Y_target2];

threshold = 0.10;

yCubicTraj = zeros(3,4);
for i = 1:4 
    yCubicTraj(i,:) = trajGen.cubic_traj(0,travelTime, Y_targets(1,i), Y_targets(2,i), 0, 0);
end

robot.interpolate_jp([90, 0, 0, 0,], 5);
pause(5);

j = 1; 
recordings = zeros(6000, 5);
detRec = zeros(626,1);
velRec = zeros(6000,4);
prev_time = 0;

tic;
while toc < travelTime
    % Record the joint position of your robot
    % Record the determinant of the Jacobian
    % Test if your robot is too close to a singularity, and stop if it is

    % Evaluate your trajectory, and send your robot to its next setpoint
    points = trajGen.eval_traj(yCubicTraj, toc);
    joints = robot.ik3001(transpose(points));
    
    J = robot.jacob3001(joints);
    
    jEE = J(1:3, 1:4); %submatrix 3x3 controls joint angles and velocities
    det_j = det(jEE * transpose(jEE));
    detRec(j,:) = det_j;
    if robot.atSingularity(threshold, joints)
    else
        robot.interpolate_jp(joints, 0.05);
    end
    tmp = measure_js(robot, true, false);
        recordings(j, 1:4) = tmp(1, :);
        recordings(j, 5) = + prev_time+toc;
        j = j+1;
end
        
taskspace = zeros(j-1,1);
for i = 1:(j-1)
    V = robot.fk3001([recordings(i,1) recordings(i,2) recordings(i,3) recordings(i,4)]);
    taskspace(i, 1) = V(2,4);
end


plot(taskspace, detRec);
xlabel('Y Position');
ylabel('Determinant of Position Jacobian');