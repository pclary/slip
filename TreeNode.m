classdef TreeNode < handle
    
    properties (SetAccess = private)
        data
        tree
        parent
        children
        null
        timestamp
    end
    
    methods
        
        
        function obj = TreeNode(tree, data, branching_factor)
            obj.data = data;
            obj.tree = tree;
            obj.parent = obj;
            obj.children = obj;
            for i = 1:branching_factor
                obj.children(i) = obj;
            end
            obj.null = true;
            obj.timestamp = uint32(0);
        end
        
        
        function c = newChild(obj, data)
            c = obj.tree.null_node;
            for i = 1:numel(obj.children)
                if obj.children(i).null
                    c = obj.tree.new();
                    obj.children(i) = c;
                end
            end
            if ~c.null
                c.data = data;
            end
        end
        
        
        function new(obj)
            obj.null = false;
        end
        
        
        function delete(obj)
            obj.tree.delete(obj);
        end
        
        
    end
    
    methods (Access = private)
        
    end
    
end
