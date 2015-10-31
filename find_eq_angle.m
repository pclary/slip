function th_eq = find_eq_angle(y0, dx0, leq0, params)

leqfun = @(t, Y, leq0) leq0;
zerofun = @(th0) stance_sim(th0, y0, dx0, leq0, leqfun, params) + th0;
th_eq = fzero(zerofun, 0);
