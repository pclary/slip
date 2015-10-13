classdef PlanarBipedController < matlab.System
    % Matlab system object implementing a Raibert-style planar biped
    % controller
    
    properties
        body_mass = 10;
        body_inertia = 1;
        foot_mass = 1;
        leg_stiffness = 1e3;
        leg_damping = 10;
        gravity = 9.81;
    end
    
    properties (DiscreteState)
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods (Access = protected)
        function setupImpl(obj, t, Y)
            % Implement tasks that need to be performed only once,
            % such as pre-computed constants.
        end
        
        function u = stepImpl(obj, t, Y)
            leg_gains = [20; 5];
            body_gains = [2; 0.5];
            
            [body, leg_a, leg_b] = bislip_kinematics(Y);
            torque_a = pd_controller(leg_a(5), leg_a(6),  0.3, 0, leg_gains);
            torque_b = pd_controller(leg_b(5), leg_b(6), -0.3, 0, leg_gains);
            torque_body = pd_controller(body(5), body(6), 0, 0, body_gains);
            
            u = [1;
                 1;
                 torque_a - torque_body/2;
                 torque_b - torque_body/2];
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end



function out = pd_controller(x, xdot, ref, refdot, gains)
% Generic PD controller
out = gains(1)*(ref - x) + gains(2)*(refdot - xdot);
end

