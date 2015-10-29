classdef PlanarBipedController < matlab.System
    % Matlab system object implementing a Raibert-style planar biped
    % controller
    
    properties
        params = zeros(11, 1);
        target_speed = 0;
    end
    
    properties (DiscreteState)
        t_last;
        Y_last;
        traj_last;
        onground_a;
        onground_b;
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
            obj.onground_a = false;
            obj.onground_b = false;
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
            
            % Unpack params
            body_mass = obj.params(1);
            body_inertia = obj.params(2);
            foot_mass = obj.params(3);
            leg_stiffness = obj.params(4);
            leg_damping = obj.params(5);
            length_motor_inertia = obj.params(6);
            length_motor_damping = obj.params(7);
            angle_motor_inertia = obj.params(8);
            angle_motor_damping = obj.params(9);
            angle_motor_ratio = obj.params(10);
            gravity = obj.params(11);
            
            % Determine phase
            force_high_threshold = gravity*body_mass/4;
            force_low_threshold = gravity*body_mass/8;
            spring_force_a = leg_stiffness*(Y(2) - Y(3));
            spring_force_b = leg_stiffness*(Y(5) - Y(6));
            if obj.onground_a && spring_force_a < force_low_threshold
                obj.onground_a = false;
            elseif ~obj.onground_a && spring_force_a > force_high_threshold
                obj.onground_a = true;
                obj.next_leg = int32(Leg.B);
            end
            if obj.onground_b && spring_force_b < force_low_threshold
                obj.onground_b = false;
            elseif ~obj.onground_b && spring_force_b > force_high_threshold
                obj.onground_b = true;
                obj.next_leg = int32(Leg.A);
            end
            if obj.onground_a && obj.onground_b
                support_type = int32(SupportType.Double);
            elseif obj.onground_a || obj.onground_b
                support_type = int32(SupportType.Single);
            else
                support_type = int32(SupportType.Flight);
            end
                
            % Generate trajectories
            % traj: [body_angle; leg_a_l; leg_a_th; leg_b_l; leg_b_th]
            body_angle = 0;
            
            leg_f_l = 1;
            leg_s_l = 1;
            leg_f_th = 0.5;
            leg_s_th = -0.5;
            
            switch obj.next_leg
                case Leg.A
                    leg_a_l = leg_f_l;
                    leg_a_th = leg_f_th;
                    leg_b_l = leg_s_l;
                    leg_b_th = leg_s_th;
                case Leg.B
                    leg_a_l = leg_s_l;
                    leg_a_th = leg_s_th;
                    leg_b_l = leg_f_l;
                    leg_b_th = leg_f_th;
                otherwise
                    error('Invalid value for next_leg: %d', obj.next_leg);
            end
            traj = [body_angle; leg_a_l; leg_a_th; leg_b_l; leg_b_th];
            
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
            body_gains = 10*[1 0.1];
            leg_length_gains = 10000*[1 0.1];
            leg_angle_gains = 100*[1 0.1];
            gains = [body_gains; leg_length_gains; leg_angle_gains; leg_length_gains; leg_angle_gains];
            
            err = traj - Y([1 2 4 5 7]);
            errdot = trajdot - Ydot([1 2 4 5 7]);
            
            umod = pd_controller(err, errdot, gains);
            
            % Set control outputs
            % u: [length_motor_a_force; angle_motor_a_torque;
            %     length_motor_b_force; angle_motor_b_torque]
            u = [umod(2);
                 umod(3) - umod(1)/2;
                 umod(4);
                 umod(5) - umod(1)/2];
            
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

