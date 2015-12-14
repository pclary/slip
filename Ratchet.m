classdef Ratchet < matlab.System
    
    properties
        reset_steps = 10;
    end
    
    properties (DiscreteState)
        last;
        directions;
        reset_countdowns;
    end
    
    methods (Access = protected)
        function setupImpl(obj, y)
            obj.last = y;
            obj.directions = zeros(size(y));
            obj.reset_countdowns = zeros(size(y));
        end
        
        function u = stepImpl(obj, y)
            if any(isnan(obj.last))
                obj.last = y;
            end
            
            diffs = y - obj.last;
            diffs(obj.directions > 0 & diffs < 0) = 0;
            diffs(obj.directions < 0 & diffs > 0) = 0;
            
            obj.reset_countdowns(diffs == 0) = obj.reset_countdowns(diffs == 0) - 1;
            obj.directions(obj.reset_countdowns <= 0) = 0;
            obj.reset_countdowns(diffs ~= 0) = obj.reset_steps;
            obj.directions(diffs ~= 0) = diffs(diffs ~= 0);
            
            u = obj.last + diffs;
            obj.last = u;
        end
        
        function resetImpl(obj)
            obj.last(:) = NaN;
            obj.directions(:) = 0;
            obj.reset_countdowns(:) = 0;
        end
    end
end
