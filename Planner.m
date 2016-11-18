classdef Planner < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-1;
        Ts_sim = 1e-3;
        env = Environment();
        ground_data = zeros(1, 5);
        rollout_depth = 3;
    end

    properties
        target_dx = 0;
        energy_injection = 0;
        phase_rate = 1.5;
        max_stride = 1;
        step_height = 0.1;
    end
    
    properties (Access = private)
        tree
        rollout_node
    end
    
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.tree = Tree(SimulationState(), 1000, 100);
        end
        
        
        function [cparams] = stepImpl(obj, X, cstate)
            cparams = ControllerParams();
            cparams.target_dx = obj.target_dx;
            cparams.energy_injection = obj.energy_injection;
            cparams.phase_rate = obj.phase_rate;
            cparams.max_stride = obj.max_stride;
            cparams.step_height = obj.step_height;
%             goal = Goal();
%             goal.dx = obj.target_dx;
%             
%             cparams = obj.tree.nodes(1).data.cparams;
%             
%             % Calculate leg phases at next planner timestep
%             phase_inc = obj.Ts * cparams.phase_rate;
%             phase_right_next = cstate.phase.right + phase_inc;
%             phase_left_next = cstate.phase.left + phase_inc;
%             
%             % Check whether next planner timestep will start a new step
%             if phase_right_next >= 1 || phase_left_next >= 1 || ...
%                     cstate.phase.right == 0 || cstate.phase.left == 0
%                 % Project one planer timestep forward with current parameters
%                 [Xp, cstatep] = biped_sim(X, cstate, cparams, obj.Ts, obj.Ts_sim, obj.env, obj.ground_data);
%                 
%                 % Choose the new parameters with the highest estimated Q value
%                 v_max = -inf;
%                 for i = 1:numel(obj.tree.nodes(1).children)
%                     c = obj.tree.nodes(1).children(i);
%                     if c
%                         v = obj.tree.nodes(c).data.value;
%                         if v > v_max
%                             cparams = obj.tree.nodes(c).data.cparams;
%                             v_max = v;
%                         end
%                     end
%                 end
%                 
%                 % Simulate until the end of the next step
%                 phase_next_max = max(mod(phase_right_next, 1), mod(phase_left_next, 1));
%                 t_stop = (1 - phase_next_max) / cparams.phase_rate;
%                 [Xp, cstatep] = biped_sim(Xp, cstatep, cparams, t_stop, obj.Ts_sim, obj.env, obj.ground_data);
%                 
%                 % Reset tree with predicted state as root
%                 vp = value(Xp, goal, obj.ground_data);
%                 ss = SimulationState(Xp, cstatep, cparams, GeneratorState(), vp);
%                 obj.tree.reset(ss);
%                 obj.rollout_node = uint32(1);
%             else
%                 % Otherwise, grow the tree
%                 
%                 % TODO: only pick nodes with no high-value child
%                 
%                 % Check whether max depth on current rollout has been reached
%                 if obj.tree.nodes(obj.rollout_node).depth >= obj.rollout_depth
%                     % Evaluate leaf node
%                     v = value(obj.tree.nodes(obj.rollout_node).data.X, goal, obj.ground_data);
%                     
%                     % Propogate value to parents
%                     i = obj.rollout_node;
%                     while (i > 0)
%                         % Set node value if greater than previous value
%                         if obj.tree.nodes(i).data.value < v
%                             obj.tree.nodes(i).data.value = v;
%                         end
%                         
%                         % Move to parent
%                         i = obj.tree.nodes(i).parent;
%                     end
%                     
%                     % Start new rollout
%                     obj.rollout_node = obj.tree.randDepth(obj.rollout_depth - 1);
%                 end
%                 
%                 % Expand on current rollout node
%                 n = obj.rollout_node;
%                 
%                 % Generate a set of parameters to try
%                 gstate = obj.tree.nodes(n).data.gstate;
%                 [cparams_gen, gstate] = generate_params(X, goal, obj.ground_data, gstate);
%                 obj.tree.nodes(n).data.gstate = gstate;
%                 
%                 % Simulate a step
%                 t_stop = 1 / cparams_gen.phase_rate;
%                 [Xp, cstatep] = biped_sim(obj.tree.nodes(n).data.X, obj.tree.nodes(n).data.cstate, ...
%                     cparams_gen, t_stop, obj.Ts_sim, obj.env, obj.ground_data);
%                 
%                 % Evaluate the result and add child node
%                 ss = SimulationState(Xp, cstatep, cparams_gen, GeneratorState(), -inf);
%                 c = obj.tree.addChild(n, ss);
%                 
%                 % If unable to add child node, delete the lowest value child
%                 if ~c
%                     v_min = inf;
%                     c_min = uint32(0);
%                     for i = 1:numel(obj.tree.nodes(n).children)
%                         ci = obj.tree.nodes(n).children(i);
%                         if ci
%                             vc = obj.tree.nodes(ci).data.value;
%                             if vc < v_min
%                                 v_min = vc;
%                                 c_min = ci;
%                             end
%                         end
%                     end
%                     if c_min
%                         obj.tree.deleteNode(c_min);
%                         c = obj.tree.addChild(n, ss);
%                     end
%                 end
%                 
%                 % Child is next parent for the rollout
%                 obj.rollout_node = c;
%             end
        end
        
        
        function resetImpl(obj)
            obj.tree.reset(SimulationState());
            obj.rollout_node = uint32(1);
        end
        
        
        function [sz1] = getOutputSizeImpl(~)
            sz1 = [1 1];
        end
        function [dt1] = getOutputDataTypeImpl(~)
            dt1 = 'cparams_bus';
        end
        function [cm1] = isOutputComplexImpl(~)
            cm1 = false;
        end
        function [fs1] = isOutputFixedSizeImpl(~)
            fs1 = true;
        end
    end
end
