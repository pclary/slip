classdef DoubleSupportController < matlab.System
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
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % control: [xdot_target]
            
            % traj: [body_angle; leg_front_leq; leg_front_th; 
            %   leg_back_leq; leg_back_th]
            traj = [0; 1; X(11); 1; X(17)];
            
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
            p_gains = [3e2; 1e4; 1e0; 1e4; 1e0];
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
                 umod(5) - umod(1)/2];
            
            obj.t_last = t;
        end
        
        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end
