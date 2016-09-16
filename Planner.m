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
        tree
    end
    
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.tree = Tree(SimulationState(), 100, 5);
        end
        
        
        function [cparams, v] = stepImpl(obj, X, cstate)
            
            goal = Goal();
            goal.dx = obj.target_dx;
            
            tstop = 0.5 / cparams_new.phase_rate;
            [X_pred, cstate_pred] = biped_sim(X, cstate, cparams_new, tstop, obj.Ts, obj.env, obj.ground_data);
            value_pred = value(X_pred, goal, obj.ground_data);
            obj.tree.reset(SimulationState(X_pred, cstate_pred, cparams_new, gstate, value_pred));
            
            
            
            cparams = ControllerParams();
            cparams.phase_rate = obj.phase_rate;
            cparams.target_dx = obj.target_dx;
            cparams.step_offset = obj.step_offset;
            cparams.energy_injection = obj.energy_injection;

            v = value(X, goal, obj.ground_data);
            
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
