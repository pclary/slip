classdef Tree < handle
    
    properties
        nodes
        root
        null_node
    end
    
    methods
        
        
        function obj = Tree(data, capacity, branching_factor)
            obj.null_node = TreeNode(obj, data, branching_factor);
            obj.nodes = TreeNode(obj, data, branching_factor);
            for i = 1:capacity
                obj.nodes(i) = TreeNode(obj, data, branching_factor);
                obj.nodes(i).delete();
            end
            obj.root = obj.new();
        end
        
        
        function node = new(obj)
            for i = 1:numel(obj.nodes)
                if ~obj.nodes(i).valid
                    node = obj.nodes(i);
                    node.valid = true;
                    return;
                end
            end
            node = obj.null_node;
        end
        
        
        function delete(obj, node)
            node.valid = false;
            node.parent = obj.nullnode;
            for i = 1:numel(node.children)
                if ~node.children(i).null
                    node.children(i).delete();
                end
                node.children(i) = obj.null_node;
            end
        end
        
        
        function clear(obj)
            for i = 1:numel(obj.nodes)
                obj.delete(obj.nodes(i));
            end
        end
       
        
    end
    
end
