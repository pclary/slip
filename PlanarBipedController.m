classdef PlanarBipedController < matlab.System
    % Matlab system object implementing a Raibert-style planar biped
    % controller
    
    properties
        body_mass = 0;
        body_inertia = 0;
        foot_mass = 0;
        leg_stiffness = 0;
        leg_damping = 0;
        gravity = 0;
        
        target_speed = 0;
    end
    
    properties (Nontunable)
        traj0 = zeros(5, 1);
    end
    
    properties (DiscreteState)
        tlast;
        Ylast;
        trajlast;
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods (Access = protected)
        function setupImpl(obj, t, Y)
            % Implement tasks that need to be performed only once,
            % such as pre-computed constants.
            obj.tlast = t;
            obj.Ylast = Y;
            obj.trajlast = obj.traj0;
        end
        
        function u = stepImpl(obj, t, Y)
            % Y: [body_angle; leg_a_angle; leg_a_length; leg_b_angle; leg_b_length]
            % Feedback derivatives
            dt = t - obj.tlast;
            dY = Y - obj.Ylast;
            if dt == 0
                Ydot = zeros(size(Y));
            else
                Ydot = dY/dt;
            end
            
            % Generate trajectories
            % traj: [body_angle; leg_a_angle; leg_a_length; leg_b_angle; leg_b_length]
            body_angle = 0;
            leg_a_angle = 0.1;
            leg_b_angle = -0.1;
            leg_a_length = 1;
            leg_b_length = 1;
            traj = [body_angle; leg_a_angle; leg_a_length; leg_b_angle; leg_b_length];
            
            % Trajectory derivatives
            dtraj = traj - obj.trajlast;
            if dt == 0
                trajdot = zeros(size(traj));
            else
                trajdot = dtraj/dt;
            end
            
            % PD control for trajectories
            % gains: [kp; kd]
            body_gains = [2; 0.5];
            leg_gains = [20; 5];
            torque_body  = pd_controller(traj(1) - Y(1), trajdot(1) - Ydot(1), body_gains);
            torque_leg_a = pd_controller(traj(2) - Y(2), trajdot(2) - Ydot(2), leg_gains);
            torque_leg_b = pd_controller(traj(4) - Y(4), trajdot(4) - Ydot(4), leg_gains);
            
            % Set control outputs
            u = [torque_leg_a - torque_body/2;
                 traj(3);
                 torque_leg_b - torque_body/2;
                 traj(5)];
            
            obj.tlast = t;
            obj.Ylast = Y;
            obj.trajlast = traj;
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end



function out = pd_controller(err, errdot, gains)
% Generic PD controller
out = gains(1).*err + gains(2).*errdot;
end

