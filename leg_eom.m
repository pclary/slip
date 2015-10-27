%% Two gearboxes in the same body
syms ddthb ddthoa ddthob ddthma ddthmb Tint1a Tint2a Tint1b Tint2b ddbx ddby ddla ddlb
syms Tma Tmb Tb Fexa Feya Fexb Feyb Fbx Fby
syms Im Ib b n lxa lya lxb lyb mb mf la lb dla dlb dthma dthmb dthla dthlb dthb
% motor and output angles are relative to body

ddthoaabs = ddthb + ddthoa;
ddthobabs = ddthb + ddthob;
ddthmaabs = ddthb + ddthma;
ddthmbabs = ddthb + ddthmb;
Foa = -lya*(Fexa - mf*ddbx) + lxa*(Feya - mf*ddby) - 2*(dthla + dthb)*dla*mf;
Fob = -lyb*(Fexb - mf*ddbx) + lxb*(Feyb - mf*ddby) - 2*(dthlb + dthb)*dlb*mf;
Fbxtot = Fbx - lya*Foa - lyb*Fob;
Fbytot = Fby + lxa*Foa + lxb*Fob;
Ioa = la^2*mf;
Iob = lb^2*mf;
Toa = Foa*la;
Tob = Fob*lb;
Tmanet = Tma - b*dthma;
Tmbnet = Tmb - b*dthmb;
Fla = lxa*(Fexa - mf*ddbx) + lya*(Feya - mf*ddby) + mf*la*(dthla + dthb)^2;
Flb = lxb*(Fexb - mf*ddbx) + lyb*(Feyb - mf*ddby) + mf*lb*(dthlb + dthb)^2;

eq1a = Im*ddthmaabs - (Tmanet - Tint1a);
eq2a = Ioa*ddthoaabs - (Toa + Tint2a);
eq3a = ddthma - n*ddthoa;
eq4a = Tint2a - n*Tint1a;
eq1b = Im*ddthmbabs - (Tmbnet - Tint1b);
eq2b = Iob*ddthobabs - (Tob + Tint2b);
eq3b = ddthmb - n*ddthob;
eq4b = Tint2b - n*Tint1b;
eq5 = Ib*ddthb - (Tb - Tmanet - Tmbnet + Tint1a + Tint1b - Tint2a - Tint2b);
eq6 = mb*ddbx - Fbxtot;
eq7 = mb*ddby - Fbytot;
eq8a = mf*ddla - Fla;
eq8b = mf*ddlb - Flb;

[ddthb, ddthoa, ddthob, ddthma, ddthmb, Tint1a, Tint2a, Tint1b, Tint2b, ddbx, ddby, ddla, ddlb] = ...
    solve(eq1a, eq2a, eq3a, eq4a, eq1b, eq2b, eq3b, eq4b, eq5, eq6, eq7, eq8a, eq8b, ...
    ddthb, ddthoa, ddthob, ddthma, ddthmb, Tint1a, Tint2a, Tint1b, Tint2b, ddbx, ddby, ddla, ddlb);

ddthb = simplify(ddthb);
ddthoa = simplify(ddthoa);
ddthab = simplify(ddthob);
ddbx = simplify(ddbx);
ddby = simplify(ddby);
ddla = simplify(ddla);
ddlb = simplify(ddlb);

syms Itot mbtot
[~, Itotexpr] = numden(ddthb);
[~, mbtotexpr] = numden(ddbx);
ddthb = simplify(subs(ddthb, Itotexpr, Itot));
ddthoa = simplify(subs(ddthoa, Itotexpr, Itot));
ddthob = simplify(subs(ddthob, Itotexpr, Itot));
ddbx = simplify(subs(ddbx, mbtotexpr, mbtot));
ddby = simplify(subs(ddby, mbtotexpr, mbtot));
Itotexpr = collect(Itotexpr, n)
mbtotexpr

f = [ddbx; ddby; ddthb; ddla; ddthoa; ddlb; ddthob];
v = [Tma; Tmb; Fexa; Feya; Fexb; Feyb; Tb; Fbx; Fby];
J = jacobian(f, v);
J = simplify(J)

f0 = f;
for i = 1:length(v)
    f0 = subs(f0, v(i), 0);
end
f0 = simplify(f0)

%% Automatically write function
headerstr = sprintf('function [J, f0] = get_eom(params, body, leg_a, leg_b)\n\nmb = params(1); Ib = params(2); mf = params(3);\nIm = params(8); b = params(9); n = params(10);\nla = leg_a(3); dla = leg_a(4);\nlb = leg_b(3); dlb = leg_b(4);\ndthb = body(6);\ndthla = leg_a(6); dthlb = leg_b(6);\ndthma = dthla*n; dthmb = dthlb*n;\nlxa = leg_a(11); lya = leg_a(12); lxb = leg_b(11); lyb = leg_b(12);');
Itots = ['Itot = ', char(Itotexpr), ';'];
mbtots = ['mbtot = ', char(mbtotexpr), ';'];
Js = strrep(strrep(char(J), '],', sprintf('];\n    ')), 'matrix(', '');
Js = ['J = ', Js(1:end-1), ';'];
f0s = strrep(strrep(char(f0), '],', sprintf('];\n     ')), 'matrix(', '');
f0s = ['f0 = ', f0s(1:end-1), ';'];
br = sprintf('\n');
filestr = [headerstr, br, br, Itots, br, mbtots, br, br, Js, br, br, f0s, br];
fid = fopen('get_eom.m', 'w');
fprintf(fid, filestr);
fclose(fid);
