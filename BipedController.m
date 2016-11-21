classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3;
    end
    
    properties (Access = private)
        cstate;
    end
    
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        
        function [u, cstate] = stepImpl(obj, X, cparams)
            [u, obj.cstate] = controller_step(X, obj.cstate, cparams, obj.Ts);
            cstate = obj.cstate;
        end
        
        
        function resetImpl(obj)
            obj.cstate = ControllerState();
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
