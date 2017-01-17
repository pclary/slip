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
            obj.tree = Tree(SimulationState(), 4096, 256);
            obj.env = Environment(obj.ground_data);
            obj.state_evaluator = StateEvaluator();
            obj.t = 0;
            obj.action_stack = Stack(ControllerParams(), obj.rollout_depth);
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
                pathnodes = zeros(obj.rollout_depth + 1, 1);
                pathnodes(1) = 1;
                temp_stack = Stack(ControllerParams(), obj.rollout_depth);
                n = 1;
                while any(obj.tree.nodes(n).children)
                    % Find child with highest rollout value
                    v_max = -inf;
                    n_new = 0;
                    for i = 1:numel(obj.tree.nodes(n).children)
                        c = obj.tree.nodes(n).children(i);
                        if c
                            v = obj.tree.nodes(c).data.path_value;
                            if v > v_max
                                cparams = obj.tree.nodes(c).data.cparams;
                                v_max = v;
                                n_new = c;
                            end
                        end
                    end
                    if ~n_new
                        break;
                    end
                    n = n_new;
                    pathnodes(find(~pathnodes, 1)) = n;
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
                stability = obj.state_evaluator.stability(Xp, terrain);
                goal_value = obj.state_evaluator.goal_value(Xp, goal);
                gstate = GeneratorState();
                gstate.last_cparams = cparams;
                ss = SimulationState(Xp, cstatep, cparams, gstate, stability, goal_value, -inf);
                obj.tree.reset(ss);
                obj.rollout_node = uint32(1);
            else
                % Otherwise, grow the tree
                
                % TODO: only pick nodes with no high-value child
                
                % Check whether max depth on current rollout has been reached
                if obj.tree.nodes(obj.rollout_node).depth >= obj.rollout_depth
                    % Evaluate leaf node   
                    stability = obj.tree.nodes(obj.rollout_node).data.stability;
                    goal_value = obj.tree.nodes(obj.rollout_node).data.goal_value;
                    
                    path_value = (goal_value + 1) / 2;
                    if stability < 0.5
                        path_value = min(path_value, stability);
                    end
                    obj.tree.nodes(obj.rollout_node).data.path_value = path_value;
                    
                    % Propogate value to parents
                    i = obj.tree.nodes(obj.rollout_node).parent;
                    while (i > 0)
                        % Compute potential new path value
                        % path_value retained from child
                        stability = obj.tree.nodes(i).data.stability;
                        goal_value = obj.tree.nodes(i).data.goal_value;
                        
                        decay = 0.5;
                        new_path_value = (goal_value + 1) / 2;
                        new_path_value = path_value * decay + new_path_value * (1 - decay);
                        new_path_value = min(new_path_value, path_value);
                        if stability < 0.5
                            new_path_value = min(path_value, stability);
                        end
                        
                        % Set node value if greater than previous value
                        if obj.tree.nodes(i).data.path_value < new_path_value
                            path_value = new_path_value;
                            obj.tree.nodes(i).data.path_value = path_value;
                        else
                            % Otherwise, stop backprop
                            break;
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
                
                % Run multiple simulations with slightly perturbed initial
                % states, and take the result with the lowest stability
                for i = 1:1
                    % Simulate a step
                    Xnp = Xn;
                    Xnp.body.x = Xnp.body.x + 1e-3*randn();
                    Xnp.body.y = Xnp.body.y + 1e-3*randn();
                    Xnp.body.dx = Xnp.body.dx + 1e-2*randn();
                    Xnp.body.dy = Xnp.body.dy + 1e-2*randn();
                    
                    [Xp{i}, cstatep{i}] = biped_sim_mex(Xnp, obj.tree.nodes(n).data.cstate, ...
                        obj.robot, cparams_gen, terrain, obj.Ts_tree, obj.Ts_sim);
                    
                    % Evaluate the result
                    terrainp = obj.env.getLocalTerrain(Xp{i}.body.x);
                    stability(i) = obj.state_evaluator.stability(Xp{i}, terrainp);
                    goal_value(i) = obj.state_evaluator.goal_value(Xp{i}, goal);
                end
                [~, i] = min(stability);
                Xp = Xp{i};
                cstatep = cstatep{i};
                stability = stability(i);
                goal_value = goal_value(i);
                
                if stability > 0.3
                    % If the stability is reasonably high, add it as a child and
                    % continue the rollout
                    gstate = GeneratorState();
                    gstate.last_cparams = cparams_gen;
                    ss = SimulationState(Xp, cstatep, cparams_gen, gstate, stability, goal_value, -inf);
                    c = obj.tree.addChild(n, ss);
                    
                    % If unable to add child node, delete the least stable child
                    if ~c
                        vs_min = inf;
                        c_min = uint32(0);
                        for i = 1:numel(obj.tree.nodes(n).children)
                            ci = obj.tree.nodes(n).children(i);
                            if ci
                                vsc = obj.tree.nodes(ci).data.stability;
                                if vsc <= vs_min
                                    vs_min = vsc;
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
                    if c
                        obj.rollout_node = c;
                    else
                        % Adding child failed; tree is probably full
                        obj.rollout_node = obj.tree.randDepth(obj.rollout_depth - 1);
                    end
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
