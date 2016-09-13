classdef TreeNode < handle
    
    properties (SetAccess = private)
        data
        tree
        parent
        children
        null
        id
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
            obj.id = uint32(0);
        end
        
        
        function c = addChild(obj, data)
            for i = 1:numel(obj.children)
                if obj.children(i).null
                    c = obj.tree.alloc();
                    if ~c.null
                        c.data = data;
                        c.parent = obj;
                        obj.children(i) = c;
                        return;
                    end
                end
            end
            c = obj.tree.null_node;
        end
        
        
        function construct(obj, id)
            obj.null = false;
            obj.id = id;
            obj.parent = obj.tree.null_node;
            for i = 1:numel(obj.children)
                obj.children(i) = obj.tree.null_node;
            end
        end
        
        
        function destruct(obj)
            obj.null = true;
            for i = 1:numel(obj.children)
                if ~obj.children(i).null
                    obj.children(i).destruct();
                end
            end
        end
        
    end
    
end
