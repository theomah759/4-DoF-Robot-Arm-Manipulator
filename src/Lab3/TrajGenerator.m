classdef TrajGenerator < handle
    methods(Static)
        % TODO: Fill in the arguments for these methods
        function coeffs = cubic_traj(t0, tf, q0, qf, v0, vf)
        %CUBIC_TRAJ Generates coefficients for a cubic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x4] matrix of cubic trajectory coefficients
            % YOUR CODE HERE
            % Hint: to solve the linear system of equations b = Ax,
            %       use x = A \ b

            A = [1, t0, t0^2, t0^3;
                 0, 1,  2*t0, 3*t0^2;
                 1, tf, tf^2, tf^3;
                 0, 1,  2*tf, 3*tf^2];

            b = [q0; v0; qf; vf];

            coeffs = A \ b;

        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quintic_traj(t0, tf, q0, qf, v0, vf, a0, af)
            %CUBIC_TRAJ Generates coefficients for a quintic trajectory
            % Inputs:
            %   TODO: Describe input args
            % Outputs:
            %   coeffs: a [1x6] matrix of cubic trajectory coefficients

            A = [1  t0  t0^2  t0^3    t0^4     t0^5;
                 0  1   2*t0  3*t0^2  4*t0^3   5*t0^4;
                 0  0   2     6*t0    12*t0^2  20*t0^3;
                 1  tf  tf^2  tf^3    tf^4     tf^5;
                 0  1   2*tf  3*tf^2  4*tf^3   5*tf^4;
                 0  0   2     6*tf    12*tf^2  20*tf^3];

            b = [q0; v0; a0; qf; vf; af];

            coeffs = A \ b;
        end

        function state = eval_traj(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t

        % Each row of coeff_mat = number of trajectories

            state = zeros(size(coeff_mat, 1), 1); % make the state vector nx1
        for i = 1:size(coeff_mat, 1)
            state(i) = polyval(flip(coeff_mat(i, :)), t);
        end
            
        % numCol = size(coeff_mat, 2);
        % 
        % if(numCol == 4)
        %     basis = [1, t, t^2, t^3];
        %     state = coeff_mat*basis;
        % elseif(numCol == 6)
        %     basis = [1, t, t^2, t^3, t^4, t^5];
        %     state = coeff_mat * basis;
        % else
        %     error("Wong number of inputs")
        end
        
        function velocity = eval_traj_vel(coeff_mat,t)
            velocity = zeros(size(coeff_mat, 1), 1); % make the state vector nx1
            for i = 1:size(coeff_mat, 1)
                velocity(i) = polyval(polyder(flip(coeff_mat(i, :)), t),t);
            end
        end

        function accel = eval_traj_accel(coeff_mat,t)
            coeff_mat = flip(coeff_mat);
            trajRows = size(coeff_mat, 1);
            accel = zeros(trajRows, 1);

            for i = 1:trajRows
                accel(i) = polyval(polyder(polyder(coeff_mat(i,:))),t);
            end
        end

        %% PLOT CUBIC & QUINTIC TRAJECTORIES
        function trajectoryPlots()
            clc
            
            t0 = 1;
            tf = 2;
            q0 = pi/2; % Initial position
            qf = pi/3; % Final position
            v0 = 0; % Initial velocity
            vf = 0; % Final velocity
            a0 = 0; % Initial acceleration
            af = 0.01; % Final acceleration

            t = linspace(t0,tf,100);
            stateTraj_cubic = zeros(size(t));
            stateTraj_quintic = zeros(size(t));
            stateTraj_cubic_vel = zeros(size(t)); 
            stateTraj_quintic_vel = zeros(size(t));
            stateTraj_cubic_accel = zeros(size(t)); 
            stateTraj_quintic_accel = zeros(size(t));




            coeffsTraj_cubic = TrajGenerator.cubic_traj(t0, tf, q0, qf, v0, vf);
            coeffsTraj_quintic = TrajGenerator.quintic_traj(t0, tf, q0, qf, v0, vf, a0, af);

            for j =1:length(t)
                stateTraj_cubic(j) = TrajGenerator.eval_traj(coeffsTraj_cubic',t(j));
                stateTraj_quintic(j) = TrajGenerator.eval_traj(coeffsTraj_quintic',t(j));
                stateTraj_cubic_vel(j) = TrajGenerator.eval_traj_vel(coeffsTraj_cubic',t(j));
                stateTraj_quintic_vel(j) = TrajGenerator.eval_traj_vel(coeffsTraj_quintic',t(j));
                stateTraj_cubic_accel(j) = TrajGenerator.eval_traj_accel(coeffsTraj_cubic',t(j));
                stateTraj_quintic_accel(j) = TrajGenerator.eval_traj_accel(coeffsTraj_quintic',t(j));

            end



            figure;
            grid on;
            plot(t, stateTraj_cubic, 'b', t, stateTraj_quintic, 'r', LineWidth=1);
            title('Cubic and Quintic Trajectory vs Time');
            xlabel('Time (s)');
            ylabel('Trajectory Position');
            legend('Cubic', 'Quintic');

            figure;
            grid on;
            plot(t, stateTraj_cubic_vel, 'b', t, stateTraj_quintic_vel, 'r', LineWidth=1);
            title('Cubic and Quintic Velocity vs Time');
            xlabel('Time (s)');
            ylabel('Trajectory Position');
            legend('Cubic', 'Quintic');

            figure;
            grid on;
            plot(t, stateTraj_cubic_accel, 'b', t, stateTraj_quintic_accel, 'r', LineWidth=1);
            title('Cubic and Quintic Acceleration vs Time');
            xlabel('Time (s)');
            ylabel('Trajectory Position');
            legend('Cubic', 'Quintic');
        end

    end

end