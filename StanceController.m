classdef StanceController < matlab.System
    % Stance phase controller for planar biped
    
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
        
        function u = stepImpl(obj, ~, t, X)
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_st_leq; leg_st_leqdot; leg_st_l; leg_st_ldot; leg_st_th; leg_st_thdot;
            %     leg_sw_leq; leg_sw_leqdot; leg_sw_l; leg_sw_ldot; leg_sw_th; leg_sw_thdot]
            % control: [xdot_target]
            
            % Estimate angle of legs with ground (normal = 0)
            ground_angle_st = X(5) + X(11);
            ground_angle_sw = X(5) + X(17);
            body_height = X(9)*cos(ground_angle_st);
            leg_sw_clearance = 0.1;
            leg_sw_leq_target = (body_height - leg_sw_clearance)/cos(ground_angle_sw);
            if isnan(leg_sw_leq_target)
                leg_sw_leq_target = 1;
            end
            leg_sw_leq_max = 1;
            leg_sw_leq_min = 0.5;
            leg_sw_leq_target = min(max(leg_sw_leq_target, leg_sw_leq_min), leg_sw_leq_max);
            
            % traj: [body_angle; leg_stance_leq; leg_stance_th; 
            %   leg_swing_leq; leg_swing_th]
            traj = [0; 1; X(11); leg_sw_leq_target; -X(11)];
            
            % Trajectory derivatives
            dt = t - obj.t_last;
            if any(isnan(obj.traj_last)) || dt == 0
                dtraj = zeros(size(traj));
                trajdot = zeros(size(dtraj));
            else
                dtraj = traj - obj.traj_last;
                trajdot = dtraj/dt;
            end
            
            % gains: [body_angle; leg_stance_leq; leg_stance_th; 
            %   leg_swing_leq; leg_swing_th] * [kp, kd]
            p_gains = [1e3; 1e4; 1e0; 1e3; 1e2];
            d_gains = p_gains*0.1;
            
            % PD control for trajectories
            ind = [5 7 11 13 17];
            err = traj - X(ind);
            errdot = trajdot - X(ind + 1);
            umod = p_gains.*err + d_gains.*errdot;
            
            % Set control outputs
            % u: [length_motor_a_force; angle_motor_a_torque;
            %     length_motor_b_force; angle_motor_b_torque]
            u = [umod(2);
                 umod(3) - umod(1)/2;
                 umod(4);
                 umod(5) - 0*umod(1)/2];
            
            obj.t_last = t;
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end
