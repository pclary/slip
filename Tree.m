classdef Tree < handle
    
    properties (SetAccess = private)
        nodes
    end
    
    
    methods
        
        function obj = Tree(data, capacity, branching_factor)
            obj.nodes = struct(...
                'data',     {data}, ...
                'parent',   {0}, ...
                'children', {zeros(branching_factor, 1)}, ...
                'index',    repmat({0}, capacity, 1));
            obj.alloc();
        end
        
        
        function c = addChild(obj, index, data)
            for i = 1:numel(obj.nodes(index).children)
                if ~obj.nodes(index).children(i)
                    c = obj.alloc();
                    if c
                        obj.nodes(c).data = data;
                        obj.nodes(c).parent = index;
                        obj.nodes(index).children(i) = c;
                        return;
                    end
                end
            end
            c = 0;
        end
        
        
%         function destruct(obj, index)
%             if ~index
%                 return
%             end
%             
%             p = obj.nodes(index).parent;
%             if p
%                 for i = 1:numel(obj.nodes(p).children)
%                     if obj.nodes(p).children(i) == index
%                         obj.nodes(p).children(i) = 0;
%                     end
%                 end
%             end
%             
%             for i = 1:numel(obj.nodes(index).children)
%                 obj.destruct(obj.nodes(index).children(i));
%             end
%             
%             obj.nodes(index).index = 0;
%         end
        
        
        function clear(obj)
            for i = 1:numel(obj.nodes)
                obj.nodes(i).index = 0;
            end
            obj.alloc();
        end
        
    end
    
    
    methods (Access = private)
        
        function node = alloc(obj)
            for i = 1:numel(obj.nodes)
                if ~obj.nodes(i).index
                    node = i;
                    obj.nodes(i).index = i;
                    obj.nodes(i).parent = 0;
                    obj.nodes(i).children = zeros(size(obj.nodes(i).children));
                    return;
                end
            end
            node = 0;
        end
       
    end
    
end
