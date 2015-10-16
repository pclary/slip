%%
syms mb Ib mf Ima Iml g kl bl bma bml na nl positive
syms xb yb thb lfa lfb thfa thfb leqa leqb Ffxa Ffya Ffxb Ffyb Ffra Ffta Ffrb Fftb Fbx Fby taa tla tab tlb real

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{xb},{yb},{thb},{leqa},{lfa},{thfa},{leqb},{lfb},{thfb}]; % Using ABSOLUTE angles here
dxb = fulldiff(xb, GC);
dyb = fulldiff(yb, GC);
dthb = fulldiff(thb, GC);
dleqa = fulldiff(leqa, GC);
dlfa = fulldiff(lfa, GC);
dthfa = fulldiff(thfa, GC);
dleqb = fulldiff(leqb, GC);
dlfb = fulldiff(lfb, GC);
dthfb = fulldiff(thfb, GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xfa = xb + lfa*sin(thfa + thb);
yfa = yb - lfa*cos(thfa + thb);
xfb = xb + lfb*sin(thfb + thb);
yfb = yb - lfb*cos(thfb + thb);
thmaa = thfa*na;
thmla = leqa*nl;
thmab = thfb*na;
thmlb = leqb*nl;

% 1c. Define any required velocity terms (for masses):
dxfa = fulldiff(xfa, GC);
dyfa = fulldiff(yfa, GC);
dxfb = fulldiff(xfb, GC);
dyfb = fulldiff(yfb, GC);
dthmaa = fulldiff(thmaa, GC);
dthmla = fulldiff(thmla, GC);
dthmab = fulldiff(thmab, GC);
dthmlb = fulldiff(thmlb, GC);

% 2. Kinetic Energy:
T = 1/2*mb*(dxb^2 + dyb^2) + ...
    1/2*Ib*dthb^2 + ...
    1/2*mf*(dxfa^2 + dyfa*2 + dxfb^2 + dyfb^2) + ...
    1/2*Ima*(dthmaa^2 + dthmab^2) + ...
    1/2*Iml*(dthmla^2 + dthmlb^2);

% 3. Potential Energy:
V = mb*g*yb + mf*g*(yfa + yfb) + 1/2*kl*((lfa - leqa)^2 + (lfb - leqb)^2);

% 4. Lagrangian:
L = T-V;

% 5. EOMs:
eq1 = simplify(fulldiff(diff(L, dxb), GC) - diff(L, xb))
eq2 = simplify(fulldiff(diff(L, dyb), GC) - diff(L, yb))
eq3 = simplify(fulldiff(diff(L, dthb), GC) - diff(L, thb))
eq4 = simplify(fulldiff(diff(L, dleqa), GC) - diff(L, leqa))
eq5 = simplify(fulldiff(diff(L, dlfa), GC) - diff(L, lfa))
eq6 = simplify(fulldiff(diff(L, dthfa), GC) - diff(L, thfa))
eq7 = simplify(fulldiff(diff(L, dleqb), GC) - diff(L, leqb))
eq8 = simplify(fulldiff(diff(L, dlfb), GC) - diff(L, lfb))
eq9 = simplify(fulldiff(diff(L, dthfb), GC) - diff(L, thfb))

% 6. Xi: non-conservative terms
Xi1 = 0;%Fbx + Ffxa + Ffxb
Xi2 = 0;%Fby + Ffya + Ffyb
Xi3 = 0;%lfa*Ffta + lfb*Fftb
Xi4 = 0;%-bml*nl*dthmla
Xi5 = 0;%Ffra
Xi6 = 0;%lfa*Ffta - bma*dthmaa
Xi7 = 0;%-bml*nl*dthmlb
Xi8 = 0;%Ffrb
Xi9 = 0;%lfb*Fftb - bml*dthmab


%%

syms d2xb d2yb d2thb d2leqa d2lfa d2thfa d2leqb d2lfb d2thfb real
[d2xb, d2yb, d2thb, d2leqa, d2lfa, d2thfa, d2leqb, d2lfb, d2thfb] = ...
    solve(eq1-Xi1, eq2-Xi2, eq3-Xi3, eq4-Xi4, eq5-Xi5, eq6-Xi6, ...
    eq7-Xi7, eq8-Xi8, eq9-Xi9, d2xb, d2yb, d2thb, d2leqa, d2lfa, d2thfa, d2leqb, d2lfb, d2thfb);

d2xb = simplify(d2xb)
d2yb = simplify(d2yb)
d2thb = simplify(d2thb)
d2leqa = simplify(d2leqa)
d2lfa = simplify(d2lfa)
d2thfa = simplify(d2thfa)
d2leqb = simplify(d2leqb)
d2lfb = simplify(d2lfb)
d2thfb = simplify(d2thfb)
