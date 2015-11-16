classdef LegController < matlab.System
    % Single leg controller for planar biped
    % Controls leg A
    % Rearrange inputs to control leg B
    
    properties
        Ts = 1e-3;
        params = zeros(11, 1);
        % params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
        %          length_motor_inertia; length_motor_damping; angle_motor_inertia;
        %          angle_motor_damping; angle_motor_ratio; gravity]
    end
    
    properties (Access = private)
        th_target;
        energy_last;
        energy_accumulator;
        acc_count;
        feet_latched;
        ratio_last;
        touchdown_length;
        energy_input;
        post_midstance_latched;
        dcomp_last;
        extension_length;
        comp_peak;
        step_optimizer;
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.step_optimizer = StepOptimizer();
            obj.step_optimizer.params = obj.params;
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
                obj.dcomp_last = [X(8) - X(10); X(14) - X(16)];
            end
            
            % Find midstance events either based on leg compression
            dcomp = [X(8) - X(10); X(14) - X(16)];
            compression_triggers = dcomp < 0 & obj.dcomp_last >= 0;
            obj.dcomp_last = dcomp;
            post_midstance = feet == 1 & (compression_triggers);
            midstance_events = obj.post_midstance_latched == false & post_midstance == true;
            obj.post_midstance_latched = (obj.post_midstance_latched | post_midstance) & feet ~= 0;
            
            % Use average values of gait energy and forward velocity over
            % last cycle for speed/energy regulation
            if any(midstance_events)
                obj.energy_last = obj.energy_accumulator/obj.acc_count;
                obj.ratio_last = abs(X(4))/(abs(X(2)) + abs(X(4)));
                obj.acc_count = 0;
                obj.energy_accumulator = 0;
            end
            obj.energy_accumulator = obj.energy_accumulator + get_gait_energy(X, obj.params);
            obj.acc_count = obj.acc_count + 1;
            
            % Record leg length at touchdown
            touchdown_events = obj.feet_latched == false & logical(floor(feet)) == true;
            takeoff_events = obj.feet_latched == true & logical(ceil(feet)) == false;
            obj.feet_latched(feet == 0) = false;
            obj.feet_latched(feet == 1) = true;
            if touchdown_events(1)
                obj.touchdown_length = X(9);
            end
            
            if takeoff_events(1)
                obj.step_optimizer.reset();
            end
            
            % Record compression at midstance
            if compression_triggers(1) && feet(1) == 1
                obj.comp_peak = X(7) - X(9);
            end
            
            % Angle controller
            dx = X(2);
            dx_target = control(2);
            ff = 0.1/obj.touchdown_length;
            kp = 0.2;
            obj.th_target = ff*dx + kp*(dx - dx_target);
            
            dx0 = X(2);
            dy0 = min(X(4), 0);
            leq0 = 1;
            leq_ext = obj.energy_input;
            target = control(2);
            obj.th_target = obj.step_optimizer.step(dx0, dy0, leq0, leq_ext, target);

            % Energy controller
            energy_target = control(1);
            err = energy_target - obj.energy_last;
            max_extension = 0.15;
            kp = 1e-3;
            ff = 0.05;
            obj.energy_input = min(max(kp*err + ff, 0), max_extension);
            
            % Compute sub-controlers
            % [support; mirror; touchdown]
            u_sub = zeros(3, 2);
            target_sub = zeros(3, 3);
            dtarget_sub = zeros(3, 3);
            kp_sub = zeros(3, 3);
            kd_sub = zeros(3, 3);
            [u_sub(1, :), target_sub(1, :), dtarget_sub(1, :), kp_sub(1, :), kd_sub(1, :)] ...
                = obj.support_controller(X);
            [u_sub(2, :), target_sub(2, :), dtarget_sub(2, :), kp_sub(2, :), kd_sub(2, :)] ...
                = obj.mirror_controller(X);
            [u_sub(3, :), target_sub(3, :), dtarget_sub(3, :), kp_sub(3, :), kd_sub(3, :)] ...
                = obj.touchdown_controller(X);
            
            u_support = u_sub(1, :)';
            u_mirror = u_sub(2, :)';
            u_touchdown = u_sub(3, :)';
            
            % Parameters used to interpolate between sub-controllers
            movement_dir = sign(X(2));
            % 0 before target angle, 1 close to and after
            p_angle = 1 - min(max(movement_dir*(obj.th_target - X(5) - X(11))/0.01, 0), 1);
            % 1 before support transfer, 0 after
            p_push = 1 - min(max(-movement_dir*(X(11) + X(17) + 2*X(5))/0.05, 0), 1);
            
            % Phase controllers
            % [flight_a; double_a; stance_a; flight_b; double_b; stance_b]
            p_sub2phase = [0      0         1;
                           1      0         0;
                           1      0         0;
                           0      1         0;
                           p_push 1-p_push  0;
                           0      1-p_angle p_angle];
            
            u_phase = p_sub2phase*u_sub;
            kp_phase = p_sub2phase*kp_sub;
            kd_phase = p_sub2phase*kd_sub;
            target_phase = p_sub2phase*(target_sub.*kp_sub)./kp_phase;
            dtarget_phase = p_sub2phase*(dtarget_sub.*kd_sub)./kd_phase;
            target_phase(isnan(target_phase)) = 0;
            dtarget_phase(isnan(dtarget_phase)) = 0;
            
            u_fa = u_touchdown;
            u_da = u_support;
            u_sa = u_support;
            u_fb = u_mirror;
            u_db = p_push*u_support + (1 - p_push)*u_mirror;
            u_sb = (1 - p_angle)*u_mirror + p_angle*u_touchdown;
            
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
            
            p_phase2eff = [p_fa, p_da, p_sa, p_fb, p_db, p_sb];
            u_eff = p_phase2eff*u_phase;
            kp_eff = p_phase2eff*kp_phase;
            kd_eff = p_phase2eff*kd_phase;
            target_eff = p_phase2eff*(target_phase.*kp_phase)./kp_eff;
            dtarget_eff = p_phase2eff*(dtarget_phase.*kd_phase)./kd_eff;
            target_eff(isnan(target_eff)) = 0;
            dtarget_eff(isnan(dtarget_eff)) = 0;
            
            u = u_sa*p_sa + u_fb*p_fb + u_db*p_db + u_sb*p_sb + u_fa*p_fa + u_da*p_da;
            
            
            leq = X(7);
            dleq = X(8);
            th_body = X(5);
            dth_body = X(6);
            th_a = X(11);
            dth_a = X(12);
            
            err = target_eff - [leq, th_a, th_body];
            derr = dtarget_eff - [dleq, dth_a, dth_body];
            
            u2 = ([1 0 0; 0 1 -1]*(kp_eff.*err + kd_eff.*derr)')';
            
            % Prevent ground slip
            ground_force = max(obj.params(4)*(X(9) - X(7)), 0);
            friction = 1;
            slip_margin = 2;
            torque_over = max(abs(u(2)) - X(9)*ground_force*friction/slip_margin, 0);
            u(2) = u(2) - feet(1)*torque_over;
            
            u = u';
            
%             [~, debug] = get_gait_energy(X, obj.params);
%             debug = [obj.energy_last; obj.ratio_last*100];
%             debug = obj.th_target;
            debug = obj.touchdown_length;
            if t > 0.3
                0;
            end
        end
            
        function resetImpl(obj)
            % Initialize discrete-state properties.
            obj.th_target = 0;
            obj.acc_count = 0;
            obj.energy_accumulator = 0;
            
            obj.energy_last = NaN;
            
            obj.feet_latched = [false; false];
            obj.ratio_last = 1;
            obj.touchdown_length = 1;
            
            obj.energy_input = 0;
            obj.post_midstance_latched = [false; false];
            
            obj.dcomp_last = [0; 0];
            obj.comp_peak = 0;
            
            obj.extension_length = 0;
            
            obj.step_optimizer.reset();
        end
    end
    
    methods (Access = private)
        function [u, target, dtarget, kp, kd] = support_controller(obj, X)
            % Use angle torque to keep body upright, maintain leg length
            
            leq = X(7);
            dleq = X(8);
            th_body = X(5);
            dth_body = X(6);
            th_a = X(11);
            dth_a = X(12);
            
            extension_time = 0.2;
            if any(obj.post_midstance_latched)
                extension_rate = obj.energy_input/extension_time;
                obj.extension_length = min(obj.extension_length + obj.Ts*extension_rate, obj.energy_input);
                leq_target = obj.touchdown_length + obj.extension_length;
                dleq_target = extension_rate;
            else
                obj.extension_length = 0;
                leq_target = obj.touchdown_length;
                dleq_target = 0;
            end
            
            th_a_target = 0;
            dth_a_target = 0;
            
            th_body_target = 0;
            dth_body_target = 0;
            
            target = [leq_target, th_a_target, th_body_target];
            dtarget = [dleq_target, dth_a_target, dth_body_target];
            
            err = target - [leq, th_a, th_body];
            derr = dtarget - [dleq, dth_a, dth_body];
            
            kp = [1e5, 0, 4e2];
            kd = 0.1*[4e3, 0, 60];
            
            u = ([1 0 0; 0 1 -1]*(kp.*err + kd.*derr)')';
        end
        
        function [u, target, dtarget, kp, kd] = mirror_controller(obj, X)
            % Mirror other leg angle and keep foot clear of ground
            
            leq = X(7);
            dleq = X(8);
            th_body = X(5);
            dth_body = X(6);
            th_a = X(11);
            dth_a = X(12);
            th_b = X(17);
            dth_b = X(18);
            
            [leq_target, dleq_target] = get_clearance_length(X);
            th_a_target = -th_b - 2*th_body;
            dth_a_target = -dth_b - 2*dth_body;
            
            th_body_target = 0;
            dth_body_target = 0;
            
            target = [leq_target, th_a_target, th_body_target];
            dtarget = [dleq_target, dth_a_target, dth_body_target];
            
            err = target - [leq, th_a, th_body];
            derr = dtarget - [dleq, dth_a, dth_body];
            
            kp = [4e2, 80, 0];
            kd = 0.1*[40, 15, 0];
            
            u = ([1 0 0; 0 1 -1]*(kp.*err + kd.*derr)')';
        end
        
        function [u, target, dtarget, kp, kd] = touchdown_controller(obj, X)
            % Set length to normal leg length, angle to touchdown angle
            
            leq = X(7);
            dleq = X(8);
            th_body = X(5);
            dth_body = X(6);
            th_a = X(11);
            dth_a = X(12);
            
            leq_target = min(X(7), 1);
            dleq_target = 2;
            th_a_target = obj.th_target - th_body;
            dth_a_target = 0 - dth_body;
            
            th_body_target = 0;
            dth_body_target = 0;
            
            target = [leq_target, th_a_target, th_body_target];
            dtarget = [dleq_target, dth_a_target, dth_body_target];
            
            err = target - [leq, th_a, th_body];
            derr = dtarget - [dleq, dth_a, dth_body];
            
            kp = [4e2, 80, 0];
            kd = 0.1*[40, 15, 0];
            
            u = ([1 0 0; 0 1 -1]*(kp.*err + kd.*derr)')';
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
