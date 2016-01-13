function [inground, normal, xc, yc, ic, pc] = test_ground(point, ground_data)

% Find location on ground that point is contacting
[xcc, ycc, icc, pcc] = pointxpoly(point(1), point(2), ground_data(:, 1), ground_data(:, 2));
offset_vector = [xcc(1) - point(1); ycc(1) - point(2)];
if isempty(icc)
    inground = false;
    normal = [0; 1];
    xc = 0;
    yc = 0;
    ic = 1;
    pc = 0;
    return
elseif length(icc) > 1 && pcc(1) == 1 && icc(2) == icc(1) + 1
    % Special case for corners
    gs1 = [diff(ground_data(icc(1):icc(1)+1, 1)); diff(ground_data(icc(1):icc(1)+1, 2))];
    gs2 = [diff(ground_data(icc(2):icc(2)+1, 1)); diff(ground_data(icc(2):icc(2)+1, 2))];
    gs = gs1/norm(gs1) + gs2/norm(gs2);
    ov_rot = [offset_vector(2); -offset_vector(1)];
    ground_segment = sign(dot(ov_rot, gs))*ov_rot/norm(ov_rot);
else
    ground_segment = [diff(ground_data(icc(1):icc(1)+1, 1)); diff(ground_data(icc(1):icc(1)+1, 2))];
end

% Right hand side of polyline is ground side
inground = ground_segment(1)*offset_vector(2) - ground_segment(2)*offset_vector(1) > 0;
normal = [-ground_segment(2); ground_segment(1)]/norm(ground_segment);
xc = xcc(1);
yc = ycc(1);
ic = icc(1);
pc = pcc(1);


function [xc, yc, ic, pc] = pointxpoly(xpt, ypt, xpl, ypl)
% Find closest point on polyline to a given point
dxpl = xpl(2:end) - xpl(1:end-1);
dypl = ypl(2:end) - ypl(1:end-1);
dxptpl = xpt(1) - xpl(1:end-1);
dyptpl = ypt(1) - ypl(1:end-1);

p = (dxpl.*dxptpl + dypl.*dyptpl)./(dxpl.*dxpl + dypl.*dypl);
p(p < 0) = 0;
p(p > 1) = 1;

xlpt = xpl(1:end-1) + p.*dxpl;
ylpt = ypl(1:end-1) + p.*dypl;

d2 = (xlpt - xpt).^2 + (ylpt - ypt).^2;
ic = find(d2 == min(d2));
xc = xlpt(ic);
yc = ylpt(ic);
pc = p(ic);
