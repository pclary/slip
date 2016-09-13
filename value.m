function [v, f] = value(X, goal, ground_data, weight, offset)

% Feature vector
f = [sign(X.body.dx) * (X.body.dx - goal.dx);
    ground_distance(X.body.x, X.body.y, ground_data);
    X.body.dy;
    sign(X.body.dx) * (mod(X.body.theta + pi, 2*pi) - pi);
    sign(X.body.dx) * (X.body.theta + (X.right.theta + X.left.theta) / 2);
    abs(X.right.theta - X.left.theta);
    (X.right.l + X.left.l) / 2;
    sign(X.body.dx) * X.body.dtheta;
    sign(X.body.dx) * (X.body.dtheta + (X.right.dtheta + X.left.dtheta) / 2);
    sign(X.right.theta - X.left.theta) * (X.right.dtheta - X.left.dtheta);
    (X.right.dl + X.left.dl) / 2];

v = dot(abs(f - offset), weight);


function dist = ground_distance(x, y, ground_data)

% Find the point on the ground closest to the point to test
min_dist2 = inf;
for i = 1:size(ground_data, 1) - 1
    xg = ground_data(i, 1);
    yg = ground_data(i, 2);
    dxg = ground_data(i + 1, 1) - xg;
    dyg = ground_data(i + 1, 2) - yg;
    
    % Take dot product to project test point onto line, then normalize with the
    % segment length squared and clamp to keep within line segment bounds
    dot_product = (x - xg) * dxg + (y - yg) * dyg;
    seg_length2 = (dxg * dxg) + (dyg * dyg);
    p = clamp(dot_product / seg_length2, 0, 1);
    
    % Nearest point on the line segment to the test point
    x_line = xg + (p * dxg);
    y_line = yg + (p * dyg);
    
    % Squared distance from line point to test point
    dist2 = ((x - x_line) * (x - x_line)) + ...
        ((y - y_line) * (y - y_line));
    
    % If this is a new minimum, save values
    % Ignore segments with zero length
    if dist2 < min_dist2 && seg_length2 > 0
        min_dist2 = dist2;
    end
end

dist = sqrt(min_dist2);


function out = clamp(x, lower, upper)
% CLAMP Constrain the value to be within the given bounds.
out = min(max(x, lower), upper);
