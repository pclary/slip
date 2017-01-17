classdef Stack < handle
    
    properties
        stack
        head
    end
    
    
    methods
        
        function obj = Stack(data, capacity)
            obj.stack = repmat({data}, capacity, 1);
            obj.head = uint32(1);
        end
        
        
        function push(obj, data)
            if obj.head <= numel(obj.stack)
                obj.stack{obj.head} = data;
                obj.head = obj.head + 1;
            end
        end
        
        
        function data = pop(obj)
            if obj.head > 1
                obj.head = obj.head - 1;
            end
            data = obj.stack{obj.head};
        end
        
        
        function data = peek(obj)
            data = obj.stack{max(obj.head - 1, 1)};
        end
        
        
        function clear(obj)
            obj.head = uint32(1);
        end
        
        
        function e = isempty(obj)
            e = obj.head <= 1;
        end
        
    end
    
end
