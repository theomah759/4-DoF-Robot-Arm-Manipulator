
% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        trajGen;
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
            self.trajGen = TrajGenerator();
            % self.imgProcessor = ImageProcessor();
        end



        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.

        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end

        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'}
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);

            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
            %SERVO_JP Send robot to a joint configuration
            % Inputs:
            %    q: a [1x4] vector containing the target joint positions

            % YOUR CODE HERE
            self.writeTime(0.1); % ignores interpolation
            self.writeJoints(q); % moves joints to the desired angles

            % Hint: look at lab1_base.m to see how to move your robot
            % Hint 2: don't set the travel time to 0, as that causes
            %         ~unintended behaviors~; do something small like
            %         0.001

            writeTime(self, 0.01);
            writeJoints(self, q);

        end

        function interpolate_jp(self, q, time)
            %INTERPOLATE_JP Send robot to a joint configuration over a period of time
            % Inputs:
            %    q: a [1x4] vector containing the target joint positions
            %    t: a scalar that tells how long to take to travel to the new position
            %       in milliseconds

            writeTime(self, time);
            writeJoints(self, q);
        end

        function q_curr = measure_js(robot, GETPOS, GETVEL)
            %MEASURED_JS Get the current position and velocity of the robot
            % Inputs:
            %    GETPOS: a boolean indicating whether or not to retrieve joint
            %            positions
            %    GETVEL: a boolean indicating whether or not to retrieve joint
            %            velocities
            % Outputs:
            %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
            %            GETPOS is false), and whose bottom row contains joint
            %            velocities

            % This line gets the current positions of the joints
            % (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;

            % This line gets the current joint velocities
            % (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);

            % YOUR CODE HERE
            % Hint: Only collect the data you need; serial read operations
            %       are expensive!

            q_curr = zeros(2,4); % Empty [2x4] matrix to store position and velocity values

            if GETPOS % Position is stored in the first row of the [2x4] matrix
                q_curr(1,:) = (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end

            if GETVEL % Velocity is stored in the second row of the [2x4] matrix
                q_curr(2,:) = (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            end
        end


        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
            %DH2MAT Gives the transformation matrix for a given row of a DH table
            % Inputs:
            %    dh_row: a [1x4] vector containing representing a single row of a DH
            %            table
            % Outputs:
            %    ht: a [4x4] matrix representing the transformation defined by the DH
            %        row

            % YOUR CODE HERE
            % Hint: Use the degree version of trig functions
            theta = dh_row(1); % Defines the thetaz value
            d = dh_row(2);     % Defines the dz value
            a = dh_row(3);     % Defines the ax value
            alpha = dh_row(4); % Defines the alphaA value
            ht = [cosd(theta) -sind(theta)*cosd(alpha)  sind(theta)*sind(alpha) a*cosd(theta);
                sind(theta)  cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
                0             sind(alpha)                cosd(alpha)          d;
                0                  0                          0               1];    % Defines the transformation matrix
        end

        function ht = dh2fk(self, dh_tab)
            %DH2MAT_R Calculates FK from a DH table
            % Inputs:
            %    dh_tab: a [nx4] matrix representing a DH table
            % Outputs:
            %    ht: a [4x4] matrix representing the transformation defined by the DH
            %        table

            % YOUR CODE HERE
            % Hint: Recall from lecture that the full FK of a manipulator can
            % be obtained by post multiplying all the HT matrices from each row
            % of the DH table

            % Hint 2: You just wrote a function to calculate the HT matrix
            % corresponding to one row of the DH table
            ht = eye(4); % Identity matrix
            numRows = size(dh_tab, 1); % Defines the number of rows in the DH table

            for i = 1:numRows
                frame = dh2mat(self, dh_tab(i, :)); % Makes a transformation matrix for each row in the table
                ht = ht * frame; % Multiplies the individual transformation matricies together
            end
        end

        % YOUR CODE HERE
        % Write a function fk3001 that takes in an array of four joint
        % values and uses your matlabFunction'd FK to generate the FK of
        % your robot

        % Hint: All non-static methods (which this is) should take "self" as
        % their first argument
        function ht = fk3001(self, joints)
            ht = symFK_matrix(joints(1), joints(2), joints(3), joints(4));

        end

        %% END LAB 2 CODE
        %% BEGIN LAB 3 CODE
        function qs = ik3001(self, pos)
            %IK3001_R Calculates IK for the OMX arm
            % Inputs:
            %    pos: a [nx4] matrix [x, y, z, gamma] representing a target pose in
            %    task space
            % Outputs:
            %    qs: a [1x4] matrix representing the joint values needed to reach the
            %    target position

            % YOUR CODE HERE
            % Hint: MATLAB debugger is your very best friend in the
            % entire world right now

            % if sqrt((x^2)+(y^2)+(z^2)) >= 385.4 || z<45
            %     ErrID = 'ik3001:Invalid';
            %     msg = 'Out of Range';
            %     baseException = MException(ErrID, msg);
            %     throw(baseException);
            % end

            base = 96.326;
            L1x = 24;
            L1y = 128;
            L2 = 124;
            L3 = 133.4;

            x = pos(1,1);
            y = pos(1,2);
            z = pos(1,3);
            gamma = pos(1,4);

            nAxis = sqrt(x^2+y^2)-L3*cosd(gamma);
            zPrimeAxis = z+L3*sind(gamma)-base;
            a = sqrt(nAxis^2+zPrimeAxis^2);
            L1 = sqrt(L1x^2+L1y^2);

            if (abs((L2^2 + L1^2 -a^2) / (2 * L2 * L1)) > 1)
                error("error")
            else
                A = acosd((L2^2 + L1^2 -a^2)/(2 * L2 * L1));
            end
            if(abs((a^2 + L1^2 - L2^2) / (2 * a * L1)) > 1)
                error("error");
            else
                B = acosd((a^2 + L1^2 - L2^2)/(2*a*L1));
            end

            beta = atan2d(zPrimeAxis, nAxis);
            offset = atan2d(L1x, L1y);

            if x==0 && y ==0
                q1 = 0;
            else
                q1 = atan2d(y,x);
            end

            q2 = 90 - B - beta - offset;
            q3 = 90 - A + offset;
            q4 = gamma -q3 -q2;

            qs = [q1, q2, q3, q4];


            % TODO: Before running calculations, make sure
            %   1. The length of the x, y, z vector is less than the
            %      length of the extended robot arm
            %   2. The z-value is not negative
            % Feel free to add any other invalid input conditions,
            % but these are the bare minimum
        end
        %% END LAB 3 CODE
        %% BEGIN LAB 4 CODE
        function j = jacob3001(self, qpos)
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos

            j = jacMatrix(qpos(1,1),qpos(1,2),qpos(1,3),qpos(1,4));
        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular
            %       velocity of the end effector in the base frame of the
            %       robot

            % YOUR CODE HERE

            J = self.jacob3001(qpos);
            vs = J * qvel;
            % vs = vs(:, 2);
            disp(vs);
        end

        % YOUR CODE HERE: Write a function atSingularity consumes a threshold and produces a boolean
        % indicating if the Jacobian of the robot goes below that value

        function result = atSingularity(self, threshold, qpos)
            jacobian = self.jacob3001(qpos);
            detJac = det(jacobian(1:3,1:3));
            result = detJac < threshold;
            if result
                ErrID = 'Singularity:True';
                msg = 'Singularity Reached';
                baseException = MException(ErrID, msg);
                throw(baseException);
            end
        end

        % start with current joint pos, target task pos, threshold
        % convert current joint pos with fk
        % find difference between current task and target task
        % multiply difference with inverse Jac of current pos
        % add to current task pos
        % repeat until reach threshold
        function jacobian_ik(self, threshold, qpos)
            currJoints = self.measure_js(true, false);
            guess = currJoints(1,:);

            while true
                fk = self.fk3001(guess);
                fk = [fk(1:3,4); guess(2)+guess(3)+guess(4)];
                diff = qpos - transpose(fk);

                if abs(diff) < threshold
                    break
                else
                    currJac = self.jacob3001(guess);
                    posJac = currJac(1:3,:);
                    posJac = [posJac; 0 0 0 1];
                    test = inv(((posJac * transpose(posJac)) + (4*eye(4,4))));
                    increment = transpose(posJac) * test;
                    increment = increment * transpose(diff);
                    guess = guess + transpose(increment);
                    self.interpolate_jp(guess, 5);
                    disp(guess);
                end
            end
        end

        %% END LAB 4 CODE
        %% BEGIN LAB 5 CODE
        function blocking_js_move(self, qpos, nvargs)
            arguments
                self Robot;
                qpos double;
                nvargs.time double = 2;
            end
            %BLOCKING_JS_MOVE moves the robot to a position in joint space
            %before exiting
            % Inputs:
            %   qpos: a [1x4] matrix of joint positions
            %   time: (optional): an integer representing desired travel time
            %         in seconds. Default time: 2 seconds.

            % YOUR CODE HERE
            % NOTE: this funciton should not exit until the robot has stopped
            % moving
            self.writeTime(1);
            self.writeJoints(qpos);
            pause(nvargs.time);

        end

        function blocking_ts_move(self, pos, nvargs)
            arguments
                self Robot;
                pos double;
                nvargs.time double = 5;
                nvargs.mode string = "quintic"
            end
            %BLOCKING_TS_MOVE moves the robot in a straight line in task space
            %to the target position before exiting
            % Inputs:
            %   pos: a [1x4] matrix representing a target x, y, z, alpha
            %   time (optional): an integer representing desired travel time
            %         in seconds. Default time: 2 seconds.
            %   mode (optional): a string "cubic" or "quintic" indicating what
            %                    type of trajectory to utilize

            % YOUR CODE HERE
            % NOTE: this funciton should not exit until the robot has stopped
            % moving
            currJPos = self.measure_js(true, false);
            currPos = self.fk3001(currJPos(1,:));
            theta2 = currJPos(1,2);
            theta3 = currJPos(1,3);
            theta4 = currJPos(1,4);
            gamma = theta2+theta3+theta4;
            currPos = [currPos(1:3,4); gamma];
            % Trajectory for x, y, z, and alpha with smooth position, velocity, and acceleration profiles
            %traj = zeros(4,6);
            % y_traj = zeros(1,6);
            % z_traj = zeros(1,6);
            % alpha_traj = zeros(1,6);

            if nvargs.mode == "quintic"

                for j = 1:4
                    traj(j,:) = self.trajGen.quintic_traj(0, nvargs.time, currPos(j,:), pos(1,j), 0, 0, 0, 0);
                end
            else

                for j = 1:4
                    traj(j,:) = self.trajGen.cubic_traj(0, nvargs.time, currPos(j,:), pos(1,j), 0, 0);
                end
            end

            tic; %start timer
            while toc < nvargs.time
                t = toc; % get current time
                % Evaluate the current position and orientation at time t
                points = self.trajGen.eval_traj(traj,t);
                % y_pos  = self.trajGen.eval_traj(y_traj',t);
                % z_pos  = self.trajGen.eval_traj(z_traj',t);
                % EE_pos = self.trajGen.eval_traj(alpha_traj',t);
                joints = self.ik3001(transpose(points));
                self.interpolate_jp(joints, 0.005)

                %Get joint angles from task-space position using IK fcn
                % joints = self.ik3001([x_pos, y_pos, z_pos, EE_pos]);
                % self.writeJoints(joints);

                pause(0.005); 

            end
            pause(.25)
        end

        function openGripper(self)
            pause(2)
            TravelTime = 2;
            self.writeTime(0.001);
            self.writeGripper(true);
            pause(TravelTime);

        end

        function closeGripper(self)
            pause(2)
            TravelTime = 2;
            self.writeTime(0.001);
            self.writeGripper(false);
            pause(TravelTime);

        end

        % Write a function pick_up_ball that satisfies the following
        % requirements:
        function pick_up_ball(self, pos, color, nvargs)
            %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
            % Inputs:
            %   pos: a [1x2] matrix representing the position of the ball in the XY
            %        frame of the robot
            %   color: a string indicating what color bin the ball should be placed
            %          in
            %   z_offset (optional): the z-position at which to begin the straight
            %                        vertical trajectory
            %   z_ball (optional): the z-posiiton at which to stop the vertical
            %                      trajectory and grasp the ball

            arguments
                self Robot;
                pos double;
                color string;
                nvargs.z_offset double = 100.0;
                nvargs.z_ball double = 18.0;
            end

            % joint1, joint2, joint3, alpha
            yellow = [-50, 20, -20, 75];
            red =    [-65, 15, -15, 80];
            green = [ 65, 15, -15, 80];
            orange = [ 50, 20, -20, 75];
            nextPos = zeros(0,0);

            switch lower(color)
                case 'green'
                    nextPos  = green;
                case 'yellow'
                    nextPos = yellow;
                case 'red'
                    nextPos = red;
                case 'orange'
                    nextPos = orange;
            end



            % Get x and y position of ball
            ballPos_X = pos(:,1);
            ballPos_Y = pos(:,2);

            % Open the gripper before moving EE
            self.openGripper();

            % Move EE to hover over ball position
            self.blocking_ts_move([ballPos_X ballPos_Y nvargs.z_offset 90], "time", 5, "mode", "quintic");

            %Lower EE to grab the ball
            self.blocking_ts_move([ballPos_X ballPos_Y nvargs.z_ball 90], "time", 5, "mode", "quintic");

            % Close the gripper after lowering EE
            self.closeGripper();

            % Send robot to HOME position in joint-space
            self.blocking_js_move([0, 0, 0, 0]);

            % Get joint configuration for the colored bin
            % bin_pos = colorBinsJointPos(1);

            % Send robot to the correct bin position in joint-space
            %self.blocking_js_move([bin_pos(1), bin_pos(2), bin_pos(3), bin_pos(4)]);
            self.blocking_js_move(nextPos);

            % Drop the ball and restart at HOME position
            self.openGripper();
            self.closeGripper();
            self.blocking_js_move([0, 0, 0, 0]);

        end


    end % end methods
end % end class
