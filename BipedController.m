classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3
        env = struct();
    end

    properties
        phase_rate = 1.6;
        target_dx = 0;
    end
    
    
    properties (Access = private)
        phase
        x_target
        length_trajectory
        angle_trajectory
        body_angle_trajectory
    end
    
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.phase = struct('right', 0, 'left', 0);
            obj.length_trajectory = struct(...
                'phase',  {0.0, 0.2, 0.4, 0.6, 0.8, 1.0}, ...
                'torque', {6.0, 6.0, 0.0, 0.0, 6.0, 6.0}, ...
                'target', {0.8, 0.7, 0.65, 0.7, 0.8, 0.8}, ...
                'kp',     {3e2, 3e2, 1e2, 1e2, 3e2, 3e2}, ...
                'kd',     {3e0, 3e0, 1e0, 1e0, 3e0, 3e0});
            obj.angle_trajectory = struct(...
                'phase',  {0.0, 0.4, 0.6, 0.8, 1.0}, ...
                'torque', {1.0, 0.0, 0.0, 0.0, 1.0}, ...
                'target', {0.0, 0.3, 0.6, 1.0, 0.0}, ...
                'kp',     {0.0, 1e2, 2e2, 4e2, 0.0}, ...
                'kd',     {0.0, 1e0, 2e0, 4e0, 0.0});
            obj.body_angle_trajectory = struct(...
                'phase',  {0.0, 0.4, 0.6, 1.0}, ...
                'torque', {0.0, 0.0, 0.0, 0.0}, ...
                'target', {0.0, 0.0, 0.0, 0.0}, ...
                'kp',     {-1e4, 0.0, 0.0, -1e4}, ...
                'kd',     {-1e3, 0.0, 0.0, -1e3});
        end
        
        
        function u = stepImpl(obj, t, X)
            
            % Increase leg phases with time
            obj.phase.right = obj.phase.right + obj.Ts * obj.phase_rate;
            obj.phase.left  = obj.phase.left  + obj.Ts * obj.phase_rate;
            
            if obj.phase.right >= 1 || obj.phase.left >= 1
                obj.x_target = X.body.x + 0.27 * X.body.dx + 0.1 * (X.body.dx - obj.target_dx);
            end
            
            % Limit phases to [0, 1)
            obj.phase.right = obj.phase.right - floor(obj.phase.right);
            obj.phase.left  = obj.phase.left  - floor(obj.phase.left);
            
            % Initialize torque struct
            u.right.l_eq = 0;
            u.right.theta_eq = 0;
            u.left.l_eq = 0;
            u.left.theta_eq = 0;
            
            % Phase determines leg length control directly
            u.right.l_eq = ...
                eval_trajectory(interp_trajectory(obj.length_trajectory, obj.phase.right), ...
                X.right.l_eq, X.right.dl_eq);
            u.left.l_eq = ...
                eval_trajectory(interp_trajectory(obj.length_trajectory, obj.phase.left), ...
                X.left.l_eq, X.left.dl_eq);
            
            % Angle targets are modulated by footstep target location
            horizontal_push = 0;
            
            if obj.phase.right > obj.phase.left
                leg_forward = X.right;
            else
                leg_forward = X.left;
            end
            
            target_step_angle_forward = ...
                real(asin(complex((obj.x_target - X.body.x) / leg_forward.l))) - X.body.theta;
            target_step_angle_back = 0;
            
            if obj.phase.right > obj.phase.left
                target_step_angle_right = target_step_angle_forward;
                target_step_angle_left = target_step_angle_back;
            else
                target_step_angle_right = target_step_angle_back;
                target_step_angle_left = target_step_angle_forward;
            end
            
            tvals_right = interp_trajectory(obj.angle_trajectory, obj.phase.right);
            tvals_right.torque = tvals_right.torque * horizontal_push;
            tvals_right.dtorque = tvals_right.dtorque * horizontal_push;
            tvals_right.target = tvals_right.target * target_step_angle_right;
            tvals_right.dtarget = tvals_right.dtarget * target_step_angle_right;
            u.right.theta_eq = eval_trajectory(tvals_right, X.right.theta_eq, X.right.dtheta_eq);
            
            tvals_left = interp_trajectory(obj.angle_trajectory, obj.phase.left);
            tvals_left.torque = tvals_left.torque * horizontal_push;
            tvals_left.dtorque = tvals_left.dtorque * horizontal_push;
            tvals_left.target = tvals_left.target * target_step_angle_left;
            tvals_left.dtarget = tvals_left.dtarget * target_step_angle_left;
            u.left.theta_eq = eval_trajectory(tvals_left, X.left.theta_eq, X.left.dtheta_eq);
            
            % Body target control is determined by phase and modulated by
            % leg force later
            
            
            % Modulate leg angle forces to limit foot slip
            
            
        end
        
        
        function resetImpl(obj)
            obj.phase.right = 0;
            obj.phase.left = 0.5;
            obj.x_target = 0;
        end
        
        
        function out = getOutputSizeImpl(~)
            out = [1 1];
        end
        
        function out = getOutputDataTypeImpl(~)
            out = 'control_bus';
        end
        
        function out = isOutputComplexImpl(~)
            out = false;
        end
        
        function out = isOutputFixedSizeImpl(~)
            out = true;
        end
    end
    
    
    methods (Access = private)
        
    end
end



function tvals = interp_trajectory(tpoints, phase)

i = 2;
while tpoints(i).phase < phase
    i = i + 1;
end

phase_diff = tpoints(i).phase - tpoints(i - 1).phase;
p = (phase - tpoints(i - 1).phase) / phase_diff;

tvals.phase = phase;
tvals.torque = tpoints(i - 1).torque + p * (tpoints(i).torque - tpoints(i - 1).torque);
tvals.dtorque = (tpoints(i).torque - tpoints(i - 1).torque) / phase_diff;
tvals.target = tpoints(i - 1).target + p * (tpoints(i).target - tpoints(i - 1).target);
tvals.dtarget = (tpoints(i).target - tpoints(i - 1).target) / phase_diff;
tvals.kp = tpoints(i - 1).kp + p * (tpoints(i).kp - tpoints(i - 1).kp);
tvals.kd = tpoints(i - 1).kd + p * (tpoints(i).kd - tpoints(i - 1).kd);
end


function torque = eval_trajectory(tvals, x, dx)
torque = tvals.torque + ...
    (tvals.kp * (tvals.target - x)) + ...
    (tvals.kd * (tvals.dtarget - dx));
end
        
