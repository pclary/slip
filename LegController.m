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
        footstep_target;
    end
    
    methods (Access = protected)
        function setupImpl(obj, ~, ~, ~, ~, ~)
            obj.footstep_target = 0;
        end
        
        function u = stepImpl(obj, control, t, X, ~, feet)
            % control: [xdot_target]
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % phase: [alpha; beta]
            % feet: [foot_a_contact; foot_b_contact];
            
            % Update footstep target if this foot is in the air and the
            % other is on the ground
            if feet(1) == 0 && feet(2) > 0
                dx = X(2);
                dx_target = control(1);
                ff = 0.1;
                kp = 0.1;
                stride_length = ff*dx + kp*(dx - dx_target);
                leg_b_x = X(1) + X(15)*sin(X(5) + X(17));
                obj.footstep_target = leg_b_x + stride_length;
            end
            
            % Foot on ground controller
            u_down = obj.foot_down_controller(X);
            
            % Foot in air controller
            u_up = obj.foot_up_controller(X, feet);
            
            % Interpolate between ground and air control based on ground
            % contact intensity
            u = feet(1)*u_down + (1 - feet(1))*u_up;
            
            % Output as row vector
            u = u';
        end
            
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
    
    methods (Access = private)
        function u = foot_down_controller(obj, X)
            % Use angle torque to keep body upright
            % Strong leg length gains
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            
            leq_target = 1;
            body_th_target = 0;
            dleq_target = 0;
            body_dth_target = 0;
            
            kp = [1e4; -1e2];
            kd = kp*0.1;
            
            err = [leq_target - leq; body_th_target - body_th];
            derr = [dleq_target - dleq; body_dth_target - body_dth];
            
            u = kp.*err + kd.*derr;
        end
        
        function u = foot_up_controller(obj, X, feet)
            % Separate controllers for:
            %   Flight phase when this foot is behind the other
            %   Leg is behind footstep target
            %   Leg is ahead of or near footstep target
            
            u_back = obj.foot_up_back_controller(X);
            u_mirror = obj.foot_up_mirror_controller(X);
            u_touchdown = obj.foot_up_touchdown_controller(X);
            
            % Interpolate between the three controllers
            movement_dir = sign(X(2));
            
            th_ramp_width = 0.1;
            th_a = X(11);
            th_b = X(17);
            p1 = min(max(movement_dir*(th_b - th_a)/th_ramp_width, 0), 1); % Is this leg behind other?
            p_back = p1*(1 - feet(2));
            
            x_ramp_width = 0.1;
            x_target = obj.footstep_target;
            x_leg = X(1) + X(9)*sin(X(5) + X(11));
            p2 = min(max(movement_dir*(x_target - x_leg)/x_ramp_width, 0), 1); % Is foot near or ahead of target?
            p_mirror = (1 - p_back)*p2;
            p_touchdown = (1 - p_back)*(1 - p2);
            
            u = u_back*p_back + u_mirror*p_mirror + u_touchdown*p_touchdown;
        end
        
        function u = foot_up_back_controller(obj, X)
            % Back leg in flight phase
            % Let leg swing freely, keep foot off ground
            
            leq = X(7);
            dleq = X(8);
            
            [leq_target, dleq_target] = get_clearance_length(X);
            
            kp = 1e3;
            kd = kp*0.1;
            
            err = leq_target - leq;
            derr = dleq_target - dleq;
            
            u = [kp*err + kd*derr; 0];
        end
        
        function u = foot_up_mirror_controller(obj, X)
            % Leg in air, not near footstep target
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
            
            kp = [1e3; 1e2];
            kd = kp*0.1;
            
            err = [leq_target - leq; th_a_target - th_a];
            derr = [dleq_target - dleq; dth_a_target - dth_a];
            
            u = kp.*err + kd.*derr;
        end
        
        function u = foot_up_touchdown_controller(obj, X)
            % Leg in air, near footstep target
            % Set length to normal leg length, adjust angle to hit target
            
            leq = X(7);
            dleq = X(8);
            body_th = X(5);
            body_dth = X(6);
            th = X(11);
            dth = X(12);
            th_abs = th + body_th;
            x = X(1);
            dx = X(2);
            l = X(9);
            dl = X(10);
            x_target = obj.footstep_target;
            
            leq_target = 1;
            dleq_target = 0;
            sinth = min(max((x - x_target)/l, 0), 1);
            th_a_target = -asin(sinth) - body_th;
            dth_a_target = -(dx + dl*sin(th_abs) + body_dth*l*cos(th_abs))/(l*cos(th_abs));
            
            kp = [1e3; 1e2];
            kd = kp*0.1;
            
            err = [leq_target - leq; th_a_target - th];
            derr = [dleq_target - dleq; dth_a_target - dth];
            
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
