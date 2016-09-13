classdef Tree < handle
    
    properties (SetAccess = private)
        nodes
        root
        null_node
        counter
    end
    
    
    methods
        
        function obj = Tree(data, capacity, branching_factor)
            obj.null_node = TreeNode(obj, data, branching_factor);
            obj.nodes = TreeNode(obj, data, branching_factor);
            for i = 1:capacity
                obj.nodes(i) = TreeNode(obj, data, branching_factor);
                obj.nodes(i);
            end
            obj.counter = uint32(0);
            obj.root = obj.alloc();
        end
        
        
        function node = alloc(obj)
            for i = 1:numel(obj.nodes)
                if obj.nodes(i).null
                    node = obj.nodes(i);
                    obj.counter = obj.counter + 1;
                    node.construct(obj.counter);
                    return;
                end
            end
            node = obj.null_node;
        end
        
        
        function clear(obj)
            for i = 1:numel(obj.nodes)
                obj.nodes(i).destruct();
            end
            obj.counter = uint32(0);
            obj.root = obj.alloc();
        end
       
    end
    
end
