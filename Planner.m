classdef Planner < matlab.System & matlab.system.mixin.Propagates
    
    properties (Nontunable)
        Ts = 1e-1;
        Ts_sim = 1e-3;
        Ts_tree = 0.25;
        robot = RobotParams();
        ground_data = zeros(2, 5);
        rollout_depth = 4;
    end

    properties
        target_dx = 0;
        energy_injection = 0;
        phase_rate = 1.5;
        max_stride = 1;
        step_height = 0.1;
        phase_stretch = 0;
    end
    
    properties (Access = private)
        tree
        rollout_node
        env
        state_evaluator
        t
        action_stack
    end
    
    
    methods (Access = protected)
        function setupImpl(obj)
            obj.tree = Tree(SimulationState(), 1000, 100);
            obj.env = Environment(obj.ground_data);
            obj.state_evaluator = StateEvaluator();
            obj.t = 0;
            obj.action_stack = Stack(ControllerParams(), obj.rollout_depth + 1);
        end
        
        
        function [cparams] = stepImpl(obj, X, cstate)
%             cparams = ControllerParams();
%             cparams.target_dx = obj.target_dx;
%             cparams.energy_injection = obj.energy_injection;
%             cparams.phase_rate = obj.phase_rate;
%             cparams.max_stride = obj.max_stride;
%             cparams.step_height = obj.step_height;
%             cparams.phase_stretch = obj.phase_stretch;
            goal = Goal();
            goal.dx = obj.target_dx;
            
            cparams = obj.tree.nodes(1).data.cparams;
            
            % Check whether next planner timestep will start a new tree timestep
            if obj.t + obj.Ts >= obj.Ts_tree || obj.t == 0
                % Project one planner timestep forward with current parameters
                % for delay compensation
                terrain = obj.env.getLocalTerrain(X.body.x);
                [Xp, cstatep] = biped_sim_mex(X, cstate, obj.robot, cparams, terrain, obj.Ts, obj.Ts_sim);
                
                % Store the highest value path in the action stack
                temp_stack = Stack(ControllerParams(), obj.rollout_depth + 1);
                n = 1;
                while any(obj.tree.nodes(n).children)
                    % Find child with highest rollout value
                    v_max = -inf;
                    n = obj.tree.nodes(n).children(find(obj.tree.nodes(n).children, 1));
                    for i = 1:numel(obj.tree.nodes(n).children)
                        c = obj.tree.nodes(n).children(i);
                        if c
                            v = obj.tree.nodes(c).data.rollout_value;
                            if v > v_max
                                cparams = obj.tree.nodes(c).data.cparams;
                                v_max = v;
                                n = c;
                            end
                        end
                    end
                    temp_stack.push(cparams);
                end
                
                % Reverse the temporary stack into the action stack
                obj.action_stack.clear();
                while ~temp_stack.isempty()
                    obj.action_stack.push(temp_stack.pop());
                end
                
                % Head is the next action to take, remaining actions are first
                % guesses for next planning cycle
                cparams = obj.action_stack.pop();
                
                % Simulate the upcoming tree timestep
                terrain = obj.env.getLocalTerrain(Xp.body.x);
                [Xp, cstatep] = biped_sim_mex(Xp, cstatep, obj.robot, cparams, terrain, obj.Ts_tree, obj.Ts_sim);
                
                % Reset tree with predicted state as root
                terrain = obj.env.getLocalTerrain(Xp.body.x);
                vp = obj.state_evaluator.value(Xp, goal, terrain);
                ss = SimulationState(Xp, cstatep, cparams, GeneratorState(), vp, -inf);
                obj.tree.reset(ss);
                obj.rollout_node = uint32(1);
            else
                % Otherwise, grow the tree
                
                % TODO: only pick nodes with no high-value child
                
                % Check whether max depth on current rollout has been reached
                if obj.tree.nodes(obj.rollout_node).depth >= obj.rollout_depth
                    % Evaluate leaf node
                    Xn = obj.tree.nodes(obj.rollout_node).data.X;
                    terrain = obj.env.getLocalTerrain(Xn.body.x);
                    v = obj.state_evaluator.value(Xn, goal, terrain);
                    
                    % Propogate value to parents
                    i = obj.rollout_node;
                    while (i > 0)
                        % Set node value if greater than previous value
                        if obj.tree.nodes(i).data.rollout_value < v
                            obj.tree.nodes(i).data.rollout_value = v;
                        end
                        
                        % Move to parent
                        i = obj.tree.nodes(i).parent;
                    end
                    
                    % Start new rollout
                    obj.rollout_node = obj.tree.randDepth(obj.rollout_depth - 1);
                end
                
                % Expand on current rollout node
                n = obj.rollout_node;
                
                % Generate a set of parameters to try
                gstate = obj.tree.nodes(n).data.gstate;
                Xn = obj.tree.nodes(n).data.X;
                terrain = obj.env.getLocalTerrain(Xn.body.x);
                [cparams_gen, gstate] = generate_params(Xn, goal, terrain, gstate, obj.action_stack);
                obj.tree.nodes(n).data.gstate = gstate;
                
                % Simulate a step
                [Xp, cstatep] = biped_sim_mex(Xn, obj.tree.nodes(n).data.cstate, ...
                    obj.robot, cparams_gen, terrain, obj.Ts_tree, obj.Ts_sim);
                
                % Evaluate the result
                terrainp = obj.env.getLocalTerrain(Xp.body.x);
                vp = obj.state_evaluator.value(Xp, goal, terrainp);
                
                if vp > 0.3
                    % If the value is reasonably high, add it as a child and
                    % continue the rollout
                    ss = SimulationState(Xp, cstatep, cparams_gen, GeneratorState(), vp, -inf);
                    c = obj.tree.addChild(n, ss);
                    
                    % If unable to add child node, delete the lowest value child
                    if ~c
                        v_min = inf;
                        c_min = uint32(0);
                        for i = 1:numel(obj.tree.nodes(n).children)
                            ci = obj.tree.nodes(n).children(i);
                            if ci
                                vc = obj.tree.nodes(ci).data.rollout_value;
                                if vc < v_min
                                    v_min = vc;
                                    c_min = ci;
                                end
                            end
                        end
                        if c_min
                            obj.tree.deleteNode(c_min);
                            c = obj.tree.addChild(n, ss);
                        end
                    end
                    
                    % Child is next parent for the rollout
                    obj.rollout_node = c;
                else
                    % If the value is too low, start a new rollout
                    obj.rollout_node = obj.tree.randDepth(obj.rollout_depth - 1);
                end
            end
            
            % Increment tree timestep clock
            obj.t = mod(obj.t + obj.Ts, obj.Ts_tree);
        end
        
        
        function resetImpl(obj)
            obj.tree.reset(SimulationState());
            obj.rollout_node = uint32(1);
            obj.env.ground_data = obj.ground_data;
            obj.t = 0;
            obj.action_stack.clear();
        end
        
        
        function [sz1] = getOutputSizeImpl(~)
            sz1 = [1 1];
        end
        function [dt1] = getOutputDataTypeImpl(~)
            dt1 = 'controller_params_bus';
        end
        function [cm1] = isOutputComplexImpl(~)
            cm1 = false;
        end
        function [fs1] = isOutputFixedSizeImpl(~)
            fs1 = true;
        end
    end
end
