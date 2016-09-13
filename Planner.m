classdef Planner < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-3;
        env = Environment();
        ground_data = zeros(1, 5);
    end

    properties
        phase_rate = 1.5;
        target_dx = 0;
        step_offset = 0;
        energy_injection = 0;
    end
    
    properties (Access = private)
    end
    
    
    methods (Access = protected)
        function setupImpl(obj)
        end
        
        
        function [cparams, v] = stepImpl(obj, X, cstate)
            cparams = ControllerParams();
            cparams.phase_rate = obj.phase_rate;
            cparams.target_dx = obj.target_dx;
            cparams.step_offset = obj.step_offset;
            cparams.energy_injection = obj.energy_injection;
            
            goal = Goal();
            goal.dx = obj.target_dx;
            offset = [-0.2478;
                0.7751;
                -0.0021;
                -0.0809;
                0.0113;
                0.2552;
                0.7772;
                0.0027;
                0.0072;
                0.0187;
                -0.0019];
            weight = 1./[0.4794;
                0.0003;
                0.0206;
                0.0050;
                0.0042;
                0.0372;
                0.0003;
                1.3190;
                1.0232;
                9.6877;
                0.0861];
            [v, f] = value(X, goal, obj.ground_data, weight, offset);
            
        end
        
        
        function resetImpl(obj)
        end
        
        
        function [sz1, sz2] = getOutputSizeImpl(~)
            sz1 = [1 1];
            sz2 = [1 1];
        end
        function [dt1, dt2] = getOutputDataTypeImpl(~)
            dt1 = 'cparams_bus';
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
end
