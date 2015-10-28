%% Full bislip EOMs with motor inertia, damping
syms ddbx ddby ddthb ddleqa ddla ddthoa ddleqb ddlb ddthob Tint1a Tint2a Tint1b Tint2b real
syms Fla Tma Flb Tmb Fbx Fby Tb Fefxa Fefya Fefxb Fefyb real
syms mb Ib mf ks bs Ilm blm Iam bam n g real
syms xb dxb yb dyb thb dthb leqa dleqa la dla thoa dthoa leqb dleqb lb dlb thob dthob real
syms ldirxa ldirya ldirxb ldiryb real

% Leg direction unit vectors
ldira = [ldirxa; ldirya];
ldirb = [ldirxb; ldiryb];
thdirxa = -ldirya;
thdirya = ldirxa;
thdirxb = -ldiryb;
thdiryb = ldirxb;
thdira = [thdirxa; thdirya];
thdirb = [thdirxb; thdiryb];

% Absolute angles
dthma = n*dthoa;
dthmb = n*dthob;
ddthma = n*ddthoa;
ddthmb = n*ddthob;
dthoaabs = dthb + dthoa;
dthobabs = dthb + dthob;
ddthoaabs = ddthb + ddthoa;
ddthobabs = ddthb + ddthob;
ddthmaabs = ddthb + ddthma;
ddthmbabs = ddthb + ddthmb;

% Spring force magnitude (compression positive)
Fsma = ks*(leqa - la) + bs*(leqa - la);
Fsmb = ks*(leqb - lb) + bs*(leqb - lb);

% Angle torque force magnitude
Ftma = Tint2a/la;
Ftmb = Tint2b/lb;

% Foot forces
Fefa = [Fefxa; Fefya];
Fefb = [Fefxb; Fefyb];
Fsa = ldira*Fsma;
Fsb = ldirb*Fsmb;
Fta = thdira*Ftma;
Ftb = thdirb*Ftmb;
Ffg = [0; -g*mf];
Ffanet = Fefa + Fsa + Fta + Ffg;
Ffbnet = Fefb + Fsb + Ftb + Ffg;

% Body forces
Fb = [Fbx; Fby];
Fbnet = Fb - Fsa - Fsb - Fta - Ftb;

% Foot kinematics
ddb = [ddbx; ddby];
ddfa = ddb + (ddla - la*dthoaabs^2)*ldira + (2*dla*dthoaabs + la*ddthoaabs)*thdira;
ddfb = ddb + (ddlb - lb*dthobabs^2)*ldirb + (2*dlb*dthobabs + lb*ddthobabs)*thdirb;

% Equilibirum length forces
Fleqa = Fla - blm*dleqa - Fsma;
Fleqb = Flb - blm*dleqb - Fsmb;

% Motor airgap net torque
Tmanet = Tma - bam*dthma;
Tmbnet = Tmb - bam*dthmb;

% Equations to be solved
eq1a = Iam*ddthmaabs - (Tmanet - Tint1a);
eq3a = Tint2a - n*Tint1a;
eq4a = mf*ddfa(1) - Ffanet(1);
eq5a = mf*ddfa(2) - Ffanet(2);
eq6a = Ilm*ddleqa - Fleqa;
eq1b = Iam*ddthmbabs - (Tmbnet - Tint1b);
eq3b = Tint2b - n*Tint1b;
eq4b = mf*ddfb(1) - Ffbnet(1);
eq5b = mf*ddfb(2) - Ffbnet(2);
eq6b = Ilm*ddleqb - Fleqb;
eq7 = Ib*ddthb - (Tb - Tmanet - Tmbnet + Tint1a + Tint1b - Tint2a - Tint2b);
eq8 = mb*ddbx - Fbnet(1);
eq9 = mb*ddby - Fbnet(2);

[ddbx, ddby, ddthb, ddleqa, ddla, ddthoa, ddleqb, ddlb, ddthob, Tint1a, Tint2a, Tint1b, Tint2b] = ...
    solve(eq1a, eq3a, eq4a, eq5a, eq6a, eq1b, eq3b, eq4b, eq5b, eq6b, eq7, eq8, eq9, ...
    ddbx, ddby, ddthb, ddleqa, ddla, ddthoa, ddleqb, ddlb, ddthob, Tint1a, Tint2a, Tint1b, Tint2b);

ddbx = simplify(ddbx);
ddby = simplify(ddby);
ddthb = simplify(ddthb);
ddleqa = simplify(ddleqa);
ddla = simplify(ddla);
ddthoa = simplify(ddthoa);
ddleqa = simplify(ddleqa);
ddlb = simplify(ddlb);
ddthob = simplify(ddthob);

syms den real
[~, denexpr] = numden(ddthb)
ddbx = simplify(subs(ddbx, denexpr, den));
ddby = simplify(subs(ddby, denexpr, den));
ddthb = simplify(subs(ddthb, denexpr, den));
ddleqa = simplify(subs(ddleqa, denexpr, den));
ddla = simplify(subs(ddla, denexpr, den));
ddthoa = simplify(subs(ddthoa, denexpr, den));
ddleqa = simplify(subs(ddleqa, denexpr, den));
ddlb = simplify(subs(ddlb, denexpr, den));
ddthob = simplify(subs(ddthob, denexpr, den));

% Get jacobian and constant part
f = [ddbx ddby ddthb ddleqa ddla ddthoa ddleqb ddlb ddthob]';
v = [Fla Tma Flb Tmb Fbx Fby Tb Fefxa Fefya Fefxb Fefyb]';
J = jacobian(f, v);
J = simplify(J)

f0 = f;
for i = 1:length(v)
    f0 = subs(f0, v(i), 0);
end
f0 = simplify(f0)

%% Automatically write function
headerstr = sprintf('function [J, f0] = get_eom(params, X)\n\nmb = params(1); Ib = params(2);\nmf = params(3); ks = params(4); bs = params(5);\nIlm = params(6); blm = params(7);\nIam = params(8); bam = params(9); n = params(10);\ng = params(11);\nxb = X(1); dxb = X(2); yb = X(3); dyb = X(4); thb = X(5); dthb = X(6);\nleqa = X(7); dleqa = X(8); la = X(9); dla = X(10); thoa = X(11); dthoa = X(12);\nleqb = X(13); dleqb = X(14); lb = X(15); dlb = X(16); thob = X(17); dthob = X(18);\nldirxa = sin(thb + thoa); ldirya = -cos(thb + thoa);\nldirxb = sin(thb + thob); ldiryb = -cos(thb + thob);');

dens = ['den = ', char(denexpr), ';'];
Js = strrep(strrep(char(J), '],', sprintf('];\n    ')), 'matrix(', '');
Js = ['J = ', Js(1:end-1), ';'];
f0s = strrep(strrep(char(f0), '],', sprintf('];\n     ')), 'matrix(', '');
f0s = ['f0 = ', f0s(1:end-1), ';'];
br = sprintf('\n');
filestr = [headerstr, br, br, dens, br, br, Js, br, br, f0s, br];
fid = fopen('get_eom.m', 'w');
fprintf(fid, filestr);
fclose(fid);
