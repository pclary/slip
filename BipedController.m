classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3;
        noise = 0;
    end
    
    properties (Access = private)
        cstate;
        reset_flag;
        noiseval;
    end
    
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        
        function [u, cstate] = stepImpl(obj, X, cparams)
            if obj.reset_flag
                obj.cstate.right.foot_x_last = X.body.x;
                obj.cstate.left.foot_x_last  = X.body.x;
                obj.reset_flag = false;
            end
            [u, obj.cstate] = controller_step(X, obj.cstate, cparams, obj.Ts);
            cstate = obj.cstate;
            
            f = 1e-2;
            obj.noiseval = (1 - f) * obj.noiseval + f * obj.noise * randn(4, 1);
            
            u.right.l_eq     = u.right.l_eq + obj.noiseval(1);
            u.right.theta_eq = u.right.theta_eq + obj.noiseval(2);
            u.left.l_eq      = u.left.l_eq + obj.noiseval(3);
            u.left.theta_eq  = u.left.theta_eq + obj.noiseval(4);

        end
        
        
        function resetImpl(obj)
            obj.cstate = ControllerState();
            obj.reset_flag = true;
            obj.noiseval = zeros(4, 1);
        end
        
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.cstate = obj.cstate;
            s.reset_flag = obj.reset_flag;
        end
        
        
        function loadObjectImpl(obj, s, wasLocked)
            obj.cstate = s.cstate;
            obj.reset_flag = s.reset_flag;
            loadObjectImpl@matlab.System(obj, s, wasLocked);
        end
        
        
        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
        end
        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'control_bus';
            dt2 = 'controller_state_bus';
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
end
