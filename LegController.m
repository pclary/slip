classdef LegController < matlab.System
    % Single leg controller for planar biped
    % Controls leg A
    % Rearrange inputs to control leg B
    
    properties
        params = zeros(11, 1);
        % params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
        %          length_motor_inertia; length_motor_damping; angle_motor_inertia;
        %          angle_motor_damping; angle_motor_ratio; gravity]
    end
    
    properties (DiscreteState)
        th_target;
    end
    
    methods (Access = protected)
        function setupImpl(obj, ~, ~, ~, ~, ~)
            obj.th_target = 0;
        end
        
        function [u, debug] = stepImpl(obj, control, t, X, phase, feet)
            % control: [xdot_target]
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % phase: [alpha; beta]
            %   alpha: bias towards leg A or leg B, [-1, 1]
            %   betap: proportion total leg forces, depends on alpha, [0, 1]
            %   beta: unwrapped betap, [0, 3)
            %     0 is flight, A to front
            %     1 is double support, A in front
            %     2 is flight, B to front
            %     3 is double support, B in front
            % feet: [foot_a_contact; foot_b_contact];
            
            % Raibert-style angle control
            dx = X(2);
            dx_target = control(1);
            ff = 0.09;
            kp = 0.05;
            obj.th_target = ff*dx + kp*(dx - dx_target);
            
            % Compute sub-controlers
            u_support = obj.support_controller(X);
            u_mirror = obj.mirror_controller(X);
            u_touchdown = obj.touchdown_controller(X);
            u_push = obj.push_controller(X);
            
            % Parameters used to interpolate between sub-controllers
            movement_dir = sign(X(2));
            pushvel = dx_target;
            % 0 before target angle, 1 close to and after
            p_angle = 1 - min(max(movement_dir*(obj.th_target - X(5) - X(11))/0.01, 0), 1);
            % 1 before pushvel, 0 after
            p_speed = 1 - min(max(movement_dir*(X(2) - pushvel)/0.1, 0), 1);
            % 1 before support transfer, 0 after
            p_pushangle = 1 - min(max((X(11) + X(17) + 2*X(5))/0.05, 0), 1);
            p_push = min(max(p_speed + p_pushangle, 0), 1);
            
            % Phase controllers
            u_sa = u_support;
            u_fb = u_mirror;
            u_db = p_push*u_push + (1 - p_push)*u_mirror;
            u_sb = (1 - p_angle)*u_mirror + p_angle*u_touchdown;
            u_fa = u_touchdown;
            u_da = u_support;
            
            % Phase interpolation
            alpha = phase(1);
            beta = phase(2);
            betap = abs(mod(beta + 1, 2) - 1);
            
            p_sa = min(max(alpha, 0), 1);
            p_sb = min(max(-alpha, 0), 1);
            if beta < 2
                p_fb = 0;
                p_db = 0;
                p_fa = (1 - betap)*(1 - abs(alpha));
                p_da = betap*(1 - abs(alpha));
            else
                p_fb = (1 - betap)*(1 - abs(alpha));
                p_db = betap*(1 - abs(alpha));
                p_fa = 0;
                p_da = 0;
            end
            
            u = u_sa*p_sa + u_fb*p_fb + u_db*p_db + u_sb*p_sb + u_fa*p_fa + u_da*p_da;
            
            % Output as row vector
            u = u';
            
            debug = obj.th_target;
            if t > 0.15
                0;
            end
        end
            
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
    
    methods (Access = private)
        function u = support_controller(obj, X)
            % Use angle torque to keep body upright, maintain leg length
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            
            leq_target = 1;
            body_th_target = 0;
            dleq_target = 0;
            body_dth_target = 0;
            
            kp = [4e4; -1e3];
            kd = kp.*[0.02; 0.1];
            
            err = [leq_target - leq; body_th_target - body_th];
            derr = [dleq_target - dleq; body_dth_target - body_dth];
            
            u = kp.*err + kd.*derr;
        end
        
        function u = mirror_controller(obj, X)
            % Mirror other leg angle and keep foot clear of ground
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            th_a = X(11);
            dth_a = X(12);
            th_b = X(17);
            dth_b = X(18);
            
            [leq_target, dleq_target] = get_clearance_length(X);
            th_a_target = -th_b - 2*body_th;
            dth_a_target = -dth_b - 2*body_dth;
            
            
            kp = [4e3; 4e3];
            kd = kp.*[0.03; 0.1];
            
            err = [leq_target - leq; th_a_target - th_a];
            derr = [dleq_target - dleq; dth_a_target - dth_a];
            
            u = kp.*err + kd.*derr;
        end
        
        function u = touchdown_controller(obj, X)
            % Set length to normal leg length, angle to touchdown angle
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            th_a = X(11);
            dth_a = X(12);
            
            leq_target = 1;
            dleq_target = 0;
            th_a_target = obj.th_target - body_th;
            dth_a_target = 0 - body_dth;
            
            kp = [4e3; 4e3];
            kd = kp.*[0.03; 0.1];
            
            err = [leq_target - leq; th_a_target - th_a];
            derr = [dleq_target - dleq; dth_a_target - dth_a];
            
            u = kp.*err + kd.*derr;
        end
        
        function u = push_controller(obj, X)
            % Push forward with back leg in double support
            % Currently the same as support, need to figure out how to make
            % push work correctly
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            
            leq_target = 1.0;
            body_th_target = 0;
            dleq_target = 0;
            body_dth_target = 0;
            
            kp = [4e4; -1e3];
            kd = kp.*[0.02; 0.1];
            
            err = [leq_target - leq; body_th_target - body_th];
            derr = [dleq_target - dleq; body_dth_target - body_dth];
            
            u = kp.*err + kd.*derr;
        end
    end
end

        
function [l, dl] = get_clearance_length(X)
% Get leg length required to clear ground

ground_clearance = 0.1;

y = X(3);
dy = X(4);
th = X(5) + X(11);
dth = X(6) + X(12);

l = (y - ground_clearance)/cos(th);
dl = dy/cos(th) + dth*sin(th)*(y - ground_clearance)/cos(th)^2;

l_min = 0.3;
l_max = 1;

if isnan(l)
    l = 1;
end

l = min(max(l, l_min), l_max);
end
