classdef FlightController < matlab.System
    % Flight phase controller for planar biped
    
    properties
        params = zeros(11, 1);
        % params: [body_mass; body_inertia; foot_mass; leg_stiffness; leg_damping; 
        %          length_motor_inertia; length_motor_damping; angle_motor_inertia;
        %          angle_motor_damping; angle_motor_ratio; gravity]
    end
    
    properties (DiscreteState)
        t_last;
        traj_last;
    end
    
    methods (Access = protected)
        function setupImpl(obj, ~, t, ~)
            obj.t_last = t;
            obj.traj_last = NaN*ones(5, 1);
        end
        
        function u = stepImpl(obj, control, t, X)
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % control: [xdot_target]
            
            xdot = X(2);
            xdot_target = control(1);
            
            ff_vel = 0.07;
            kp_vel = 0.2;
            
            target_angle = ff_vel*xdot_target - kp_vel*(xdot_target - xdot);
            
            % traj: [body_angle; leg_front_leq; leg_front_th; 
            %   leg_back_leq; leg_back_th]
            traj = [0; 1; target_angle; X(13); -target_angle];
            
            % Trajectory derivatives
            dt = t - obj.t_last;
            if any(isnan(obj.traj_last)) || dt == 0
                dtraj = zeros(size(traj));
                trajdot = zeros(size(dtraj));
            else
                dtraj = traj - obj.traj_last;
                trajdot = dtraj/dt;
            end
            
            % gains: [body_angle; leg_front_leq; leg_front_th; 
            %   leg_back_leq; leg_back_th] * [kp, kd]
            p_gains = [0; 1e3; 3e2; 1e3; 1e2];
            d_gains = p_gains*0.1;
            
            % PD control for trajectories
            ind = [5 7 11 13 17];
            err = traj - (X(ind) + [0; 0; X(5); 0; X(5)]);
            errdot = trajdot - (X(ind + 1) + [0; 0; X(6); 0; X(6)]);
            umod = p_gains.*err + d_gains.*errdot;
            
            % Set control outputs
            % u: [length_motor_a_force; angle_motor_a_torque;
            %     length_motor_b_force; angle_motor_b_torque]
            u = [umod(2);
                 umod(3) - umod(1)/2;
                 umod(4);
                 umod(5) - umod(1)/2];
            
            obj.t_last = t;
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end
