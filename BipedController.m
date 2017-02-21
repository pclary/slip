classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3;
    end
    
    properties (Access = private)
        cstate;
        reset_flag;
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
        end
        
        
        function resetImpl(obj)
            obj.cstate = ControllerState();
            obj.reset_flag = true;
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
