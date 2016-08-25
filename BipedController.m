classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3
    end

    properties
        phase_rate = 1.6;
        target_dx = 0;
        step_offset = 0;
        energy_injection = 0;
    end
    
    
    properties (Access = private)
        cstate;
    end
    
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        
        function [u, dbg] = stepImpl(obj, t, X)
            
            cparams = ControllerParams();
            cparams.phase_rate = obj.phase_rate;
            cparams.target_dx = obj.target_dx;
            cparams.step_offset = obj.step_offset;
            cparams.energy_injection = obj.energy_injection;

            
            [u, obj.cstate] = controller_step(X, obj.cstate, cparams, obj.Ts);
            
            dbg = [0, 0];
        end
        
        
        function resetImpl(obj)
            obj.cstate = ControllerState();
        end
        
        
        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 2];
            
        end
        
        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'control_bus';
            dt2 = 'double';
        end
        
        function [cm1, cm2] = isOutputComplexImpl(~)
            cm1 = false;
            cm2 = false;
        end
        
        function [fs1, fs2] = isOutputFixedSizeImpl(~)
            fs1 = true;
            fs2 = true;
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


function out = clamp(x, l, h)
out = min(max(x, l), h);
end
        
