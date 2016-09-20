classdef Planner < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-1;
        Ts_sim = 1e-3;
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
            
            cparams = obj.tree.root().data.cparams;
            
            % Calculate leg phases at next planner timestep
            phase_inc = obj.Ts * cparams.phase_rate;
            phase_right_next = cstate.phase.right + phase_inc;
            phase_left_next = cstate.phase.left + phase_inc;
            
            % Check whether next planner timestep will start a new step
            if phase_right_next >= 1 || phase_left_next >= 1 || ...
                    cstate.phase.right == 0 || cstate.phase.left == 0
                % Project one planer timestep forward with current parameters
                [Xp, cstatep] = biped_sim(X, cstate, cparams, obj.Ts, obj.Ts_sim, obj.env, obj.ground_data);
                
                % Choose the new parameters with the highest estimated Q value
                q_max = -inf;
                for i = 1:numel(obj.tree.root().children)
                    c = obj.tree.root().children(i);
                    if c
                        q = obj.tree.nodes(c).value;
                        if q > q_max;
                            cparams = obj.tree.nodes(c).cparams;
                            q_max = q;
                        end
                    end
                end
                
                % Simulate until the end of the next step
                phase_next_max = max(mod(phase_right_next, 1), mod(phase_left_next, 1));
                t_stop = (1 - phase_next_max) / cparams.phase_rate;
                [Xp, cstatep] = biped_sim(Xp, cstatep, cparams, t_stop, obj.Ts_sim, obj.env, obj.ground_data);
                
                % Reset tree with predicted state as root
                qp = value(Xp, goal, obj.ground_data);
                ss = SimulationState(Xp, cstatep, cparams, GeneratorState(), qp);
                obj.tree.reset(ss);
            else
                % Otherwise, grow the tree
            end
            
            v = value(X, goal, obj.ground_data);
        end
        
        
        function resetImpl(obj)
            obj.tree.reset(SimulationState());
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
