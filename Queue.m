classdef Queue < handle
    
    properties
        queue
        head
        size
    end
    
    
    methods
        
        function obj = Queue(data, capacity)
            obj.queue = repmat({data}, capacity, 1);
            obj.head = uint32(1);
            obj.size = uint32(0);
        end
        
        
        function push(obj, data)
            if obj.size < numel(obj.queue)
                obj.queue{mod(obj.head + obj.size - 1, numel(obj.queue)) + 1} = data;
                obj.size = obj.size + 1;
            end
        end
        
        
        function data = pop(obj)
            data = obj.queue{obj.head};
            if obj.size > 0
                obj.head = mod(obj.head, numel(obj.queue)) + 1;
                obj.size = obj.size - 1;
            end
        end
        
        
        function data = peek(obj)
            data = obj.queue{obj.head};
        end
        
        
        function clear(obj)
            obj.size = uint32(0);
        end
        
        
        function e = isempty(obj)
            e = obj.size < 1;
        end
        
    end
    
end
