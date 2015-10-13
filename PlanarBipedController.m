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
        u0 = zeros(4, 1);
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
            obj.trajlast = obj.u0;
        end
        
        function u = stepImpl(obj, t, Y)
            % Y: [body_angle; leg_a_angle; leg_a_length; leg_b_angle; leg_b_length]
            dt = t - obj.tlast;
            dY = Y - obj.Ylast;
            if dt == 0
                Ydot = zeros(size(Y));
            else
                Ydot = dY/dt;
            end
            
            % PD gains
            leg_gains = 20*[20; 5];
            body_gains = [2; 0.5];
            
            thamp = 0.3;
            lamp = 0.2;
            freq = 0.2*sqrt(obj.leg_stiffness/obj.body_mass);
            ta = freq*t;
            tb = freq*t + pi;
            tha  = thamp*sin(ta);
            dtha = thamp*cos(ta);
            thb  = thamp*sin(tb);
            dthb = thamp*cos(tb);
            la = 1 - max(lamp*cos(ta), -1);
            lb = 1 - max(lamp*cos(tb), -1);
            
            torque_a = pd_controller(Y(2), Ydot(2), tha, dtha, leg_gains);
            torque_b = pd_controller(Y(4), Ydot(4), thb, dthb, leg_gains);
            torque_body = pd_controller(Y(1), Ydot(1), 0, 0, body_gains);
            
            u = [la;
                 lb;
                 torque_a - torque_body/2;
                 torque_b - torque_body/2];
             
            ulimit_upper = [1.5; 1.5; 100; 100];
            ulimit_lower = [0.5; 0.5; -100; -100];
            u = min(max(u, ulimit_lower), ulimit_upper);
            
            obj.tlast = t;
            obj.Ylast = Y;
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

