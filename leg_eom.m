%%
syms xb(t) yb(t) thb(t) xbdot(t) ybdot(t) thbdot(t) xbddot ybddot thbddot 
syms l(t) th(t) ldot(t) thdot(t) lddot thddot F T M I

x = xb + l*sin(thb + th)
y = yb - l*cos(thb + th)

xdot = diff(x, t);
xdot = subs(subs(subs(subs(subs(xdot, diff(l), ldot), diff(th), thdot), diff(xb), xbdot), diff(yb), ybdot), diff(thb), thbdot)
ydot = diff(y, t);
ydot = subs(subs(subs(subs(subs(ydot, diff(l), ldot), diff(th), thdot), diff(xb), xbdot), diff(yb), ybdot), diff(thb), thbdot)

xddot = diff(xdot, t);
xddot = subs(subs(subs(subs(subs(xddot, diff(l), ldot), diff(th), thdot), diff(xb), xbdot), diff(yb), ybdot), diff(thb), thbdot);
xddot = subs(subs(subs(subs(subs(xddot, diff(ldot), lddot), diff(thdot), thddot), diff(xbdot), xbddot), diff(ybdot), ybddot), diff(thbdot), thbddot)
yddot = diff(ydot, t);
yddot = subs(subs(subs(subs(subs(yddot, diff(l), ldot), diff(th), thdot), diff(xb), xbdot), diff(yb), ybdot), diff(thb), thbdot);
yddot = subs(subs(subs(subs(subs(yddot, diff(ldot), lddot), diff(thdot), thddot), diff(xbdot), xbddot), diff(ybdot), ybddot), diff(thbdot), thbddot)


eq1 = F*sin(th)/M + T*cos(th)/I*l - xddot
eq2 = -F*cos(th)/M + T*sin(th)/I*l - yddot

[lddot, thddot] = solve(eq1, eq2, lddot, thddot);
lddot = simplify(lddot);
thddot = simplify(thddot);

syms l1 ldot1 th1 thdot1 thb1 thbdot1
lddot = subs(subs(subs(subs(subs(subs(lddot, l, l1), ldot, ldot1), th, th1), thdot, thdot1), thb, thb1), thbdot, thbdot1);
thddot = subs(subs(subs(subs(subs(subs(thddot, l, l1), ldot, ldot1), th, th1), thdot, thdot1), thb, thb1), thbdot, thbdot1);


syms l ldot th thdot thb thbdot
lddot = subs(subs(subs(subs(subs(subs(lddot, l1, l), ldot1, ldot), th1, th), thdot1, thdot), thb1, thb), thbdot1, thbdot)
thddot = subs(subs(subs(subs(subs(subs(thddot, l1, l), ldot1, ldot), th1, th), thdot1, thdot), thb1, thb), thbdot1, thbdot)

%% Gearbox with motor inertia
syms thmddot thoddotrel thbddot Im Ib Io Tm Tb To Tint1 Tint2 n

thoddot = thbddot + thoddotrel;
eq1 = Im*thmddot - (Tm - Tint1);
eq2 = Ib*thbddot - (Tb - Tm + Tint1 - Tint2);
eq3 = Io*thoddot - (To + Tint2);
eq4 = (thmddot - thbddot) - (n*(thoddot - thbddot));
eq5 = Tint2 - (n*Tint1);

[thmddot, thbddot, thoddotrel, Tint1, Tint2] = ...
    solve(eq1, eq2, eq3, eq4, eq5, thmddot, thbddot, thoddotrel, Tint1, Tint2);

thbddot = simplify(thbddot)
thoddotrel = simplify(thoddotrel)

%% Two gearboxes in the same body
syms ddthma ddthoa ddthmb ddthob ddthb Im Ib Ioa Iob Tma Tmb Tb Toa Tint1a Tint2a Tbb Tob Tint1b Tint2b n
% motor and output angles are relative to body

ddthoaabs = ddthb + ddthoa;
ddthobabs = ddthb + ddthob;
ddthmaabs = ddthb + ddthma;
ddthmbabs = ddthb + ddthmb;

eq1a = Im*ddthmaabs - (Tma - Tint1a);
eq2a = Ioa*ddthoaabs - (Toa + Tint2a);
eq3a = ddthma - n*ddthoa;
eq4a = Tint2a - n*Tint1a;
eq1b = Im*ddthmbabs - (Tmb - Tint1b);
eq2b = Iob*ddthobabs - (Tob + Tint2b);
eq3b = ddthmb - n*ddthob;
eq4b = Tint2b - n*Tint1b;
eq5 = Ib*ddthb - (Tb - Tma - Tmb + Tint1a + Tint1b - Tint2a - Tint2b);

[ddthb, ddthoa, ddthob, ddthma, ddthmb, Tint1a, Tint1b, Tint2a, Tint2b] = ...
    solve(eq1a, eq2a, eq3a, eq4a, eq1b, eq2b, eq3b, eq4b, eq5, ...
    ddthb, ddthoa, ddthob, ddthma, ddthmb, Tint1a, Tint1b, Tint2a, Tint2b);

ddthb = simplify(ddthb);
ddthoa = simplify(ddthoa);
ddthab = simplify(ddthob);

syms Itot
[~, Itotexpr] = numden(ddthb)
ddthb = simplify(subs(ddthb, Itotexpr, Itot))
ddthoa = simplify(subs(ddthoa, Itotexpr, Itot))
ddthob = simplify(subs(ddthob, Itotexpr, Itot))
