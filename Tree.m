classdef Tree < handle
    
    properties (SetAccess = private)
        nodes
    end
    
    properties (Access = private)
        stack
    end
    
    
    methods
        
        function obj = Tree(data, capacity, branching_factor)
            % Allocate max number of nodes
            obj.nodes = struct(...
                'data',     {data}, ...
                'parent',   {uint32(0)}, ...
                'children', {uint32(zeros(branching_factor, 1))}, ...
                'index',    repmat({uint32(0)}, capacity, 1));
            
            % Allocate root node
            obj.alloc();
            
            % Allocate stack for recursive deletion
            obj.stack = Stack(uint32([1, 1]), capacity + 1);
        end
        
        
        function r = root(~)
            % Root node is always at index 1
            r = uint32(1);
        end
        
        
        function c = addChild(obj, index, data)
            % Look for unused child slot on given node
            for i = 1:numel(obj.nodes(index).children)
                if ~obj.nodes(index).children(i)
                    % Node has room for another child
                    c = obj.alloc();
                    if c
                        % Allocation succeeded, set up new child node
                        obj.nodes(c).data = data;
                        obj.nodes(c).parent = index;
                        obj.nodes(index).children(i) = c;
                        return;
                    end
                end
            end
            
            % Allocation failed or max branching factor reached
            c = uint32(0);
        end
        
        
        function del(obj, index)
            % Clear child index from parent node, if applicable
            p = obj.nodes(index).parent;
            if p
                for i = 1:numel(obj.nodes(p).children)
                    if obj.nodes(p).children(i) == index
                        obj.nodes(p).children(i) = uint32(0);
                    end
                end
            end
            
            % Recursively null child nodes
            obj.stack.clear();
            i = index;
            c = uint32(1);
            obj.stack.push([i, c]);
            while ~obj.stack.isempty();
                if c > numel(obj.nodes(i).children)
                    % End of child list
                    obj.nodes(i).index = uint32(0);
                    ic = obj.stack.pop();
                    i = ic(1);
                    c = ic(2) + 1;
                elseif obj.nodes(i).children(c)
                    % Child index is valid
                    obj.stack.push([i, c]);
                    i = obj.nodes(i).children(c);
                    c = uint32(1);
                else
                    % Child index is null; proceed to next child
                    c = c + 1;
                end
            end
        end
        
        
        function reset(obj, data)
            % Null all nodes
            for i = 1:numel(obj.nodes)
                obj.nodes(i).index = uint32(0);
            end
            
            % Allocate a new root and set data
            obj.alloc();
            obj.nodes(obj.root()).data = data;
        end
        
    end
    
    
    methods (Access = private)
        
        function node = alloc(obj)
            % Find a null node
            for i = uint32(1:numel(obj.nodes))
                if ~obj.nodes(i).index
                    % Clear null node and make it valid
                    node = i;
                    obj.nodes(i).index = i;
                    obj.nodes(i).parent = uint32(0);
                    obj.nodes(i).children = uint32(zeros(size(obj.nodes(i).children)));
                    return;
                end
            end
            % Tree is at full capacity
            node = uint32(0);
        end
       
    end
    
end
