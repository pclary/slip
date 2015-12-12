classdef LegController < matlab.System
    % Single leg controller for planar biped
    % Controls leg A
    % Rearrange inputs in order to control leg B
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Block Parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
        Ts = 1e-3;
        params = zeros(11, 1);
        kp_ground = zeros(3, 1);
        kd_ground = zeros(3, 1);
        kp_air = zeros(3, 1);
        kd_air = zeros(3, 1);
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (Access = private)
        step_optimizer;
        
        th_target;
        energy_input;
        
        energy_last;
        energy_accumulator;
        energy_accumulator_count;
        
        feet_latched;
        touchdown_length;
        post_midstance_latched;
        extension_length;
        
        err_last;
        kp_last;
        forces_last;
        dforces_last;
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab System Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.step_optimizer = StepOptimizer();
            obj.step_optimizer.params = obj.params;
        end
        
        
        function [u, target, kp, debug] = stepImpl(obj, control, t, X, phase, feet, forces)
            % control: [energy_target; ratio_target]
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % phase: [alpha; beta]
            %   alpha: bias towards leg A or leg B, [-1, 1]
            %   betap: proportion total leg forces, depends on alpha, [0, 1]
            %   beta: unwrapped betap, [0, 4)
            %     0 is flight, A to front
            %     1 is double support, A in front
            %     2 is flight, B to front
            %     3 is double support, B in front
            % feet: [foot_a_contact; foot_b_contact];
            % forces: [leg_a_force; leg_b_force];
            % params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping;
            %          length_motor_inertia; length_motor_damping; angle_motor_inertia;
            %          angle_motor_damping; angle_motor_ratio; gravity]
            
            % Initialization
            if isnan(obj.energy_last)
                obj.energy_last = get_gait_energy(X, obj.err_last, obj.kp_last, obj.params);
            end
            
            % Find midstance events based on leg compression
            dforces = (forces - obj.forces_last)/obj.Ts;
            compression_triggers = forces < 0 & obj.dforces_last >= 0;
            post_midstance = feet == 1 & (compression_triggers);
            midstance_events = obj.post_midstance_latched == false & post_midstance == true;
            obj.post_midstance_latched = (obj.post_midstance_latched | post_midstance) & feet ~= 0;
            
            % Use average values of gait energy and forward velocity over
            % last cycle for speed/energy regulation
            if any(midstance_events)
                obj.energy_last = obj.energy_accumulator/obj.energy_accumulator_count;
                obj.energy_accumulator_count = 0;
                obj.energy_accumulator = 0;
            end
            obj.energy_accumulator = obj.energy_accumulator + get_gait_energy(X, obj.err_last, obj.kp_last, obj.params);
            obj.energy_accumulator_count = obj.energy_accumulator_count + 1;
            
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
            
            % Angle controller
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
            
            % Get trajectories from subcontrollers and interpolate
            [target, dtarget, kp, kd] = obj.subcontroller_interpolation(X, phase);
            
            leq = X(7);
            dleq = X(8);
            th_body = mod(X(5) + pi, 2*pi) - pi;
            dth_body = X(6);
            th_a = X(11);
            dth_a = X(12);
            
            %
            target = [1 0 0];
            dtarget = [0 0 0];
            kp = obj.kp_air;
            kd = obj.kd_air;
            %
            
            err = target - [leq, th_a, th_body];
            derr = dtarget - [dleq, dth_a, dth_body];
            
            u = [1 0 0; 0 1 -1]*(kp.*err + kd.*derr)';
            
            % Prevent ground slip
            ground_force = max(obj.params(4)*(X(7) - X(9)), 0);
            friction = 1;
            slip_margin = 2;
            torque_over = max(abs(u(2)) - X(9)*ground_force*friction/slip_margin, 0);
            u(2) = u(2) - feet(1)*torque_over;
            
            obj.err_last = err;
            obj.kp_last = kp;
            obj.forces_last = forces;
            obj.dforces_last = dforces;
            
%             [~, debug] = get_gait_energy(X, obj.err_last, obj.kp_last, obj.params);
%             debug = obj.th_target;
            debug = 0;
            if t > 0.3
                0;
            end
        end
        
            
        function resetImpl(obj)
            obj.step_optimizer.reset();
            
            obj.th_target = 0;
            obj.energy_input = 0;
            
            obj.energy_accumulator_count = 0;
            obj.energy_accumulator = 0;
            obj.energy_last = NaN;
            
            obj.feet_latched = [false; false];
            obj.touchdown_length = 1;
            obj.post_midstance_latched = [false; false];
            obj.extension_length = 0;
            
            obj.err_last = [0 0 0];
            obj.kp_last = [0 0 0];
            obj.forces_last = [0; 0];
            obj.dforces_last = [0; 0];
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = private)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Subcontrollers
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [target, dtarget, kp, kd] = support_controller(obj, X)
            % Use angle torque to keep body upright, maintain leg length
            
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
            
            target = [leq_target, 0, 0];
            dtarget = [dleq_target, 0, 0];
            
            kp = obj.kp_ground;
            kd = obj.kd_ground;
        end
        
        
        function [target, dtarget, kp, kd] = mirror_controller(obj, X)
            % Mirror other leg angle and keep foot clear of ground
            
            th_body = X(5);
            dth_body = X(6);
            th_b = X(17);
            dth_b = X(18);
            
            [leq_target, dleq_target] = get_clearance_length(X);
            th_a_target = -th_b - 2*th_body;
            dth_a_target = -dth_b - 2*dth_body;
            
            target = [leq_target, th_a_target, 0];
            dtarget = [dleq_target, dth_a_target, 0];
            
            kp = obj.kp_air;
            kd = obj.kd_air;
        end
        
        
        function [target, dtarget, kp, kd] = touchdown_controller(obj, X)
            % Set length to normal leg length, angle to touchdown angle
            
            th_body = X(5);
            dth_body = X(6);
            
            leq_target = min(X(7), 1);
            dleq_target = 2;
            th_a_target = obj.th_target - th_body;
            dth_a_target = 0 - dth_body;
            
            target = [leq_target, th_a_target, 0];
            dtarget = [dleq_target, dth_a_target, 0];
            
            kp = obj.kp_air;
            kd = obj.kd_air;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Interpolation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function [target, dtarget, kp, kd] = subcontroller_interpolation(obj, X, phase)
            % Compute sub-controlers
            % [support; mirror; touchdown]
            target_sub = zeros(3, 3);
            dtarget_sub = zeros(3, 3);
            kp_sub = zeros(3, 3);
            kd_sub = zeros(3, 3);
            [target_sub(1, :), dtarget_sub(1, :), kp_sub(1, :), kd_sub(1, :)] ...
                = obj.support_controller(X);
            [target_sub(2, :), dtarget_sub(2, :), kp_sub(2, :), kd_sub(2, :)] ...
                = obj.mirror_controller(X);
            [target_sub(3, :), dtarget_sub(3, :), kp_sub(3, :), kd_sub(3, :)] ...
                = obj.touchdown_controller(X);
            
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
            kp_phase = p_sub2phase*kp_sub;
            kd_phase = p_sub2phase*kd_sub;
            target_phase = p_sub2phase*(target_sub.*kp_sub)./kp_phase;
            dtarget_phase = p_sub2phase*(dtarget_sub.*kd_sub)./kd_phase;
            target_phase(isnan(target_phase)) = 0;
            dtarget_phase(isnan(dtarget_phase)) = 0;
            
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
            kp = p_phase2eff*kp_phase;
            kd = p_phase2eff*kd_phase;
            target = p_phase2eff*(target_phase.*kp_phase)./kp;
            dtarget = p_phase2eff*(dtarget_phase.*kd_phase)./kd;
            target(isnan(target)) = 0;
            dtarget(isnan(dtarget)) = 0;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


function [gait_energy, energies] = get_gait_energy(X, err_eff, kp_eff, params)
m = params(1);
k = params(4);
g = params(11);
spring_a_energy = 1/2*k*(X(7) - X(9))^2;
spring_b_energy = 1/2*k*(X(13) - X(15))^2;
kinetic_energy = 1/2*m*(X(2)^2 + X(4)^2);
gravitational_energy = m*g*(X(3) - 1);
controller_energy = sum(1/2*kp_eff.*err_eff.^2);
energies = [spring_a_energy; spring_b_energy; kinetic_energy; gravitational_energy; controller_energy; 0];
gait_energy = sum(energies);
energies(6) = gait_energy;
end
