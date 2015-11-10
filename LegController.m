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
        dx_last;
        energy_last;
        dx_accumulator;
        energy_accumulator;
        acc_count;
        feet_latched;
        ratio_last;
        touchdown_length;
        energy_input;
        post_midstance_latched;
        angles_last;
    end
    
    methods (Access = protected)
        function setupImpl(obj, ~, ~, X, ~, feet)
            obj.th_target = 0;
            obj.acc_count = 0;
            obj.dx_accumulator = 0;
            obj.energy_accumulator = 0;
            
            obj.dx_last = X(2);
            obj.energy_last = NaN;
            
            obj.feet_latched = [false; false];
            obj.ratio_last = 1;
            obj.touchdown_length = 1;
            
            obj.energy_input = 0;
            obj.post_midstance_latched = [false; false];
            
            obj.angles_last = [X(11); X(17)];
        end
        
        function [u, debug] = stepImpl(obj, control, t, X, phase, feet)
            % control: [energy_target; ratio_target]
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
            
            % Initialization
            if isnan(obj.energy_last)
                obj.energy_last = get_gait_energy(X, obj.params);
                obj.angles_last = [X(11); X(17)];
            end
            
            % Use average values of gait energy and forward velocity over
            % last cycle for speed/energy regulation
            % Use midstance to trigger new cycle
            
            angles = [X(11); X(17)];
            angle_triggers = angles*sign(X(2)) <= 0 & obj.angles_last*sign(X(2)) > 0;
            obj.angles_last = angles;
            post_midstance = feet == 1 & angle_triggers;
            midstance_triggers = obj.post_midstance_latched == false & post_midstance == true;
            obj.post_midstance_latched = (obj.post_midstance_latched | post_midstance) & feet ~= 0;
            if any(midstance_triggers)
                obj.dx_last = obj.dx_accumulator/obj.acc_count;
                obj.energy_last = obj.energy_accumulator/obj.acc_count;
                obj.ratio_last = abs(X(4))/(abs(X(2)) + abs(X(4)));
                obj.acc_count = 0;
                obj.dx_accumulator = 0;
                obj.energy_accumulator = 0;
            end
            obj.dx_accumulator = obj.dx_accumulator + X(2);
            obj.energy_accumulator = obj.energy_accumulator + get_gait_energy(X, obj.params);
            obj.acc_count = obj.acc_count + 1;
            
            % Record leg length at touchdown
            touchdown_edge = obj.feet_latched == false & logical(floor(feet)) == true;
            obj.feet_latched(feet == 0) = false;
            obj.feet_latched(feet == 1) = true;
            if touchdown_edge(1)
                obj.touchdown_length = X(9);
            end
            
            % Angle controller
            dx = obj.dx_last;
            dx_target = control(2);
            ff = 0.09/obj.touchdown_length;
            kp = 0.03;
            obj.th_target = ff*dx + kp*(dx - dx_target);

            % Energy controller
            energy_target = control(1);
            err = energy_target - obj.energy_last;
            max_extension = 0.1;
            kp = 1e-3;
            ff = 0.05;
            obj.energy_input = min(max(kp*err + ff, 0), max_extension);
            
            % Compute sub-controlers
            u_support = obj.support_controller(X);
            u_mirror = obj.mirror_controller(X);
            u_touchdown = obj.touchdown_controller(X);
            u_push = u_support;%obj.push_controller(X);
            
            % Parameters used to interpolate between sub-controllers
            movement_dir = sign(X(2));
            pushvel = 0;
            % 0 before target angle, 1 close to and after
            p_angle = 1 - min(max(movement_dir*(obj.th_target - X(5) - X(11))/0.01, 0), 1);
            % 1 before pushvel, 0 after
            p_speed = 1 - min(max(movement_dir*(X(2) - pushvel)/0.1, 0), 1);
            % 1 before support transfer, 0 after
            p_pushangle = 1 - min(max(-movement_dir*(X(11) + X(17) + 2*X(5))/0.05, 0), 1);
            p_push = p_pushangle;
            
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
            
            % Prevent ground slip
            ground_force = max(obj.params(4)*(X(9) - X(7)), 0);
            friction = 1;
            slip_margin = 2;
            torque_over = max(abs(u(2)) - X(9)*ground_force*friction/slip_margin, 0);
            u(2) = u(2) - feet(1)*torque_over;
            
            % Output as row vector
            u = u';
            
%             [~, debug] = get_gait_energy(X, obj.params);
            debug = [obj.energy_last; obj.ratio_last*100];
%             debug = obj.th_target;
            
            if t > 1.3
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
            
            stance_ramp = 0.1;
            stance_half = min(max(-X(11)/stance_ramp, 0), 1);
            leq_target = obj.touchdown_length + stance_half*obj.energy_input;
            dleq_target = 0;
            body_th_target = 0;
            body_dth_target = 0;
            
            kp = [1e5; -4e2];
            kd = kp.*[0.05; 0.1];
            
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
            
            
            kp = [4e4; 1e3];
            kd = kp.*[0.05; 0.1];
            
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
            
            leq_target = min(X(7), 1);
            dleq_target = 1;
            th_a_target = obj.th_target - body_th;
            dth_a_target = 0 - body_dth;
            
            kp = [1e4; 1e3];
            kd = kp.*[0.05; 0.1];
            
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
            
            leq_target = 1;
            body_th_target = 0;
            dleq_target = 0;
            body_dth_target = 0;
            
            kp = [1e5; -4e2];
            kd = kp.*[0.05; 0.1];
            
            err = [leq_target - leq; body_th_target - body_th];
            derr = [dleq_target - dleq; body_dth_target - body_dth];
            
            u = kp.*err + kd.*derr;
        end
    end
end

        
function [l, dl] = get_clearance_length(X)
% Get leg length required to clear ground

ground_clearance = 0.05;

y = X(3);
dy = X(4);
th = X(5) + X(11);
dth = X(6) + X(12);

l = (y - ground_clearance)/cos(th);
dl = dy/cos(th) + dth*sin(th)*(y - ground_clearance)/cos(th)^2;

l_min = 0.5;
l_max = 1;

if isnan(l)
    l = 1;
end

l = min(max(l, l_min), l_max);
end


function [gait_energy, energies] = get_gait_energy(X, params)
m = params(1);
k = params(4);
g = params(11);
spring_a_energy = 1/2*k*(X(7) - X(9))^2;
spring_b_energy = 1/2*k*(X(13) - X(15))^2;
kinetic_energy = 1/2*m*(X(2)^2 + X(4)^2);
gravitational_energy = m*g*(X(3) - 1);
energies = [spring_a_energy; spring_b_energy; kinetic_energy; gravitational_energy; 0; 0];
gait_energy = sum(energies);
energies(5) = gait_energy;
energies(6) = gait_energy - kinetic_energy;
end
