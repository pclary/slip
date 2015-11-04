%%

y0 = 0.7 + logspace(-1, 0.7, 100);
dx0 = linspace(0, 10, 100);
leq0 = 1;

th_eq = zeros(length(y0), length(dx0));

tic
for i = 1:length(y0)
    for j = 1:length(dx0)
        for k = 1:length(leq0)
            th_eq(i, j) = find_eq_angle_mex(y0(i), dx0(j), leq0(k), params);
        end
    end
end
toc

[y0g, dx0g] = ndgrid(y0, dx0);
surf(y0g, dx0g, th_eq);
xlabel('Apex height (m)');
ylabel('Forward velocity (m/s)');
zlabel('Equilibrium angle (rad)');
