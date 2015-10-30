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
        function setupImpl(obj, control, t, X)
            obj.t_last = t;
            obj.traj_last = NaN*ones(5, 1);
        end
        
        function u = stepImpl(obj, control, t, X)
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            % control: [xdot_target]
            
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
            
            xdot = X(2);
            xdot_target = control(1);
            
            ff_vel
            
            target_angle
            
            
            % traj: [body_angle; leg_front_leq; leg_front_th; 
            %   leg_back_leq; leg_back_th]
            % gains: [body_angle; leg_front_leq; leg_front_th; 
            %   leg_back_leq; leg_back_th] * [kp, kd]
            gains = [1e2*[1 0.1]; 1e3*[1 0.1]; 1e2*[1 0.1]; 1e3*[1 0.1]; 1e2*[1 0.1]];
            traj = [0; 1; target_angle; 1; -target_angle];
                    
                    
            switch support_type
                case SupportType.Flight
                case SupportType.Single
                    gains = [1e2*[1 0.1]; 1e3*[1 0.1]; 1e2*[1 0.1]; 1e4*[1 0.1]; 1e0*[1 0.1]];
                    traj = [0; 0.7; -Y(ind(7)); 1; Y(ind(7))];
                case SupportType.Double
                    gains = [1e2*[1 0.1]; 1e4*[1 0.1]; 1e0*[1 0.1]; 1e4*[1 0.1]; 1e0*[1 0.1]];
                    traj = [0; 1; Y(ind(4)); 1; Y(ind(4))];
                otherwise
                    error('Invalid value for support_type: %d', support_type);
            end
            
            % Trajectory derivatives
            if any(isnan(obj.traj_last)) || dt == 0
                dtraj = zeros(size(traj));
                trajdot = zeros(size(dtraj));
            else
                dtraj = traj - obj.traj_last;
                trajdot = dtraj/dt;
            end
            
            % PD control for trajectories
            ind2 = ind([1 2 4 5 7]);
            err = traj - Y(ind2);
            errdot = trajdot - Ydot(ind2);
            
            umod = pd_controller(err, errdot, gains);
            
            % Rearrange umod depending on next leg
            umod(uind) = umod;
            
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

