classdef StepOptimizer < matlab.System
    
    properties
        params = zeros(11, 1);
        dth = 0.1;
    end
    
    properties (DiscreteState)
        state;
        th;
        f;
        th2;
        f2;
        dx0;
        dy0;
        leq0;
        leq_ext;
    end
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        function u = stepImpl(obj, dx0, dy0, leq0, leq_ext, target)
            
            if ~any(obj.state == [0, 1])
                obj.th = 0.1*dx0;
                obj.state = 0;
            end
            
            switch obj.state
                case 0
                    % Calculate objective function at current angle
                    obj.dx0 = dx0;
                    obj.dy0 = dy0;
                    obj.leq0 = leq0;
                    obj.leq_ext = leq_ext;
                    [~, dxend, dyend] = ...
                        stance_sim(obj.th, obj.dx0, obj.dy0, obj.leq0, obj.leq_ext, obj.params);
                    obj.f = objective(dxend, dyend);
                    if isnan(obj.f)
                        obj.th = rand()*pi - pi/2;
                        obj.state = 0;
                    else
                        obj.state = 1;
                    end
                case 1
                    % Finite difference derivative of objective function,
                    % get new angle
                    obj.th2 = obj.th + obj.dth;
                    [~, dxend, dyend] = ...
                        stance_sim(obj.th2, obj.dx0, obj.dy0, obj.leq0, obj.leq_ext, obj.params);
                    obj.f2 = objective(dxend, dyend);
                    if isnan(obj.f2)
                        obj.th = rand()*pi - pi/2;
                    else
                        df = obj.f2 - obj.f;
                        thstep = (target - obj.f)/(df/obj.dth);
                        maxstep = pi/16;
                        thstep = min(max(thstep, -maxstep), maxstep);
                        obj.th = obj.th + thstep;
                    end
                    obj.state = 0;
            end
            
            u = obj.th;
        end
        
        function resetImpl(obj)
            obj.state = -1;
            obj.th = 0;
            obj.f = 0;
            obj.th2 = 0;
            obj.f2 = 0;
            obj.dx0 = 0;
            obj.dy0 = 0;
            obj.leq0 = 1;
            obj.leq_ext = 0;
        end
    end
    
    methods (Access = private)
    end
end


function f = objective(dxend, dyend)
f = atan2(dxend, dyend);
end
