classdef StepOptimizer < matlab.System
    
    properties
        params = zeros(11, 1);
        dth = 0.1;
    end
    
    properties (DiscreteState)
        state;
        th;
        thend;
        dxend;
        dyend;
        th2;
        thend2;
        dxend2;
        dyend2;
        dx0;
        dy0;
        leq0;
        leq_ext;
    end
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        function u = stepImpl(obj, dx0, dy0, leq0, leq_ext, dx_target)
            
            switch obj.state
                case 0
                    obj.th2 = obj.th + obj.dth;
                    [obj.thend2, obj.dxend2, obj.dyend2] = ...
                        stance_sim(obj.th2, obj.dx0, obj.dy0, obj.leq0, obj.leq_ext, obj.params);
                    dxend_diff = obj.dxend2 - obj.dxend;
                    thstep = (dx_target - obj.dxend)/(dxend_diff/obj.dth);
                    maxstep = pi/16;
                    thstep = min(max(thstep, -maxstep), maxstep);
                    obj.th = obj.th + thstep;
                    obj.state = 1;
                case 1
                    obj.dx0 = dx0;
                    obj.dy0 = dy0;
                    obj.leq0 = leq0;
                    obj.leq_ext = leq_ext;
                    [obj.thend, obj.dxend, obj.dyend] = ...
                        stance_sim(obj.th, obj.dx0, obj.dy0, obj.leq0, obj.leq_ext, obj.params);
                    obj.state = 0;
                otherwise
                    obj.dx0 = dx0;
                    obj.dy0 = dy0;
                    obj.leq0 = leq0;
                    obj.leq_ext = leq_ext;
                    obj.th = 0.1*obj.dx0;
                    [obj.thend, obj.dxend, obj.dyend] = ...
                        stance_sim(obj.th, obj.dx0, obj.dy0, obj.leq0, obj.leq_ext, obj.params);
                    obj.state = 0;
            end
            
            u = obj.th;
        end
        
        function resetImpl(obj)
            obj.state = -1;
            obj.th = 0;
            obj.thend = 0;
            obj.dxend = 0;
            obj.dyend = 0;
            obj.th2 = 0;
            obj.thend2 = 0;
            obj.dxend2 = 0;
            obj.dyend2 = 0;
            obj.dx0 = 0;
            obj.dy0 = 0;
            obj.leq0 = 1;
            obj.leq_ext = 0;
        end
    end
    
    methods (Access = private)
        
    end
end
