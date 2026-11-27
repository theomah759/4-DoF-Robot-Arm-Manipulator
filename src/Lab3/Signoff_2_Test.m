robot = Robot();

q0 = [281.4009 0 224.3263 0]; % Should work (home position)
qTest = [150 100 300 10]; % Should work
qTest1 = [100 0 10 90]; % Should work
qTest2 = [200 200 200 20]; % Should work
qTest3 = [0 -40 50 90];   % Cant work
qBad = [1000,1000,1000,1000]; % Camt work

q53 = [0,  -20, -70,  90]; % task2joint testing
qS4 = robot.fk3001(q53).';

% is = robot.ik3001(qTest1);
is = robot.ik3001([qS4(4,1:3) (q53(2)+q53(3)+q53(4))]); % task2joint testing
%is = robot.ik3001(qBad);
disp(is);  

fs = robot.fk3001(is);
disp(fs);

robot.interpolate_jp(is,5)