classdef BipedController < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3
        env = struct();
    end
    
    
    properties (Access = private)
    end
    
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        
        function u = stepImpl(~, t, X)
            
            u.right.l_eq = 0;
            u.right.theta_eq = 0;
            u.left.l_eq = 0;
            u.left.theta_eq = 0;
        end
        
        
        function resetImpl(~)
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
