function plot_tree(nodes, pathnodes)

% Find unused nodes
unused = ~[nodes.index];

% Get list of parents
p = double([nodes.parent]);
p = p(~unused);

% Get node values
data = [nodes.data];
stabilities = [data.stability];
stabilities = stabilities(~unused);
goal_values = [data.goal_value];
goal_values = goal_values(~unused);
path_values = [data.path_value];
path_values = path_values(~unused);

% Get child number for each node
cindex = zeros(size(p));
cindex(1) = 1;
for i = 2:length(p)
    cindex(i) = find(nodes(p(i)).children == i, 1);
end

index = 1:numel(nodes);
index = index(~unused);

[x, y] = treelayout(p);

% Make sure all nodes are drawn at the correct level
maxdepth = double(max([nodes.depth]));
yvals = 1 - ((0:maxdepth) + 1) / (maxdepth + 2);
for i = index
    y(i) = yvals(nodes(i).depth + 1);
end

figure;

for i = 1:length(p)
    color = 'r';
    if nargin > 1 && any(index(i) == pathnodes)
        color = 'b';
    end
    if p(i)
        line([x(p(i)), x(i)], [y(p(i)), y(i)], 'Color', color);
    end
    text(x(i), y(i), sprintf('%d\n%.2f\n%.2f\n%.2f', cindex(i), path_values(i), stabilities(i), goal_values(i)));
end

axis([0 1 0 1]);
