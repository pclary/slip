function th = find_eq_angle(y0, dx0, leq0, params)
% Find the touchdown angle that results in a symmetric stance phase

th = NaN;
th0 = 0.1;
dth0 = 0.1;
while ~isfinite(th) && th0 < pi/2
    th = findeq(th0, y0, dx0, leq0, params);
    th0 = th0 + dth0;
end


function th = findeq(th0, y0, dx0, leq0, params)
th = th0;
dth = 0.1;
for i = 1:8
    % Newton's method with a finite difference derivative
    f0 = stance_sim(th, y0, dx0, leq0, @leqfun, params) + th;
    if ~isfinite(f0)
        th = NaN;
        return;
    end
    
    th1 = th + dth;
    f1 = stance_sim(th1, y0, dx0, leq0, @leqfun, params) + th1;
    if ~isfinite(f1)
        th = NaN;
        return;
    end
    
    df = f1 - f0;
    thstep = -f0/(df/dth);
    maxstep = pi/6;
    thstep = min(max(thstep, -maxstep), maxstep);
    th = th + thstep;
end
if abs(thstep) > maxstep / 1000
    % Convergence failed
    th = NaN;
end


function leq = leqfun(~, ~, leq0)
leq = leq0;
