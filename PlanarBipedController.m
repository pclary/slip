classdef PlanarBipedController < matlab.System
    % Matlab system object implementing a Raibert-style planar biped
    % controller
    
    properties
        params = zeros(12, 1);
        target_speed = 0;
    end
    
    properties (DiscreteState)
        t_last;
        Y_last;
        traj_last;
        support_type;
        next_leg;
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods (Access = protected)
        function setupImpl(obj, t, Y)
            % Implement tasks that need to be performed only once,
            % such as pre-computed constants.
            obj.t_last = t;
            obj.Y_last = Y;
            obj.traj_last = NaN*ones(5, 1);
            obj.support_type = int32(SupportType.Flight);
            obj.next_leg = int32(Leg.A);
        end
        
        function u = stepImpl(obj, t, Y)
            % Y: [body_angle;
            %     leg_a_leq; leg_a_l; leg_a_th; 
            %     leg_b_leq; leg_b_l; leg_b_th]

            % Feedback derivatives
            dt = t - obj.t_last;
            dY = Y - obj.Y_last;
            if dt == 0
                Ydot = zeros(size(Y));
            else
                Ydot = dY/dt;
            end
            
            % Determine walking phase
            
            
            % Generate trajectories
            % traj: [body_angle; leg_a_l; leg_a_th; leg_b_l; leg_b_th]
            body_angle = 0;
            leg_a_l = 1 - 0.1*sin(t);
            leg_b_l = 1 - 0.1*cos(t);
            leg_a_th = 0.3 + 0.1*sin(t);
            leg_b_th = -0.3 + 0.1*cos(t);
            traj = [body_angle; leg_a_th; leg_a_l; leg_b_th; leg_b_l];
            
            % Trajectory derivatives
            if any(isnan(obj.traj_last)) || dt == 0
                dtraj = zeros(size(traj));
                trajdot = zeros(size(dtraj));
            else
                dtraj = traj - obj.traj_last;
                trajdot = dtraj/dt;
            end
            
            % PD control for trajectories
            % gains: [kp kd]
            body_gains = [2 0.5];
            leg_torque_gains = 1*[20 5];
            leg_length_gains = 10*[1 0.1];
            gains = [body_gains; leg_length_gains; leg_torque_gains; leg_length_gains; leg_torque_gains];
            
            err = traj - Y([1 2 3 5 6]);
            errdot = trajdot - Ydot([1 2 3 5 6]);
            
            umod = pd_controller(err, errdot, gains);
            
            % Set control outputs
            % u: [length_motor_a_force; angle_motor_a_torque;
            %     length_motor_b_force; angle_motor_b_torque]
            u = [umod(2);
                 umod(3) - umod(1)/2;
                 umod(4);
                 umod(5) - umod(1)/2];
%             u = [sin(t)*00000; 1; 0; 0];
            
            obj.t_last = t;
            obj.Y_last = Y;
            obj.traj_last = traj;
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end



function out = pd_controller(err, errdot, gains)
% Generic PD controller
out = gains(:, 1).*err + gains(:, 2).*errdot;
end

