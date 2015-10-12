function [body, leg_a, leg_b] = bislip_kinematics(Y, params)
%#codegen

% Make body and leg structures
body = makebody();
leg_a = makeleg();
leg_b = makeleg();

% Physical parameters
body.mass =       params(1);
body.inertia =    params(2);
leg_a.mass =      params(3);
leg_b.mass =      params(3);
leg_a.stiffness = params(4);
leg_b.stiffness = params(4);
leg_a.damping =   params(5);
leg_b.damping =   params(5);

% Break Y out
body.pos =    Y(1:2);
body.th =     Y(3);
body.dpos =   Y(4:5);
body.dth =    Y(6);
leg_a.foot =  Y(7:8);
leg_a.dfoot = Y(9:10);
leg_b.foot =  Y(11:12);
leg_b.dfoot = Y(13:14);

leg_a = leg_kinematics(leg_a, body);
leg_b = leg_kinematics(leg_b, body);


function leg = leg_kinematics(leg, body)
% Calculate lengths, derivatives, etc
leg.vec = leg.foot - body.pos;
leg.dvec = leg.dfoot - body.dpos;
leg.length = norm(leg.vec);
if leg.length ~= 0
    leg.direction = leg.vec/leg.length;
    leg.dangle = dot(ccw90(leg.direction), leg.dvec)/leg.length;
else
    leg.direction = [0; -1];
    leg.dangle = 0;
end
leg.angle = atan2(leg.direction(1), -leg.direction(2));
leg.dlength = dot(leg.dvec, leg.direction);


function out = ccw90(v)
% Rotate vector 90 degrees ccw
out = [-v(2); v(1)];


function leg = makeleg()
% Define fields of leg struct
leg.mass = 0;
leg.stiffness = 0;
leg.damping = 0;
leg.foot = [0; 0];
leg.dfoot = [0; 0];
leg.vec = [0; 0];
leg.dvec = [0; 0];
leg.length = 0;
leg.dlength = 0;
leg.angle = 0;
leg.dangle = 0;
leg.direction = [1; 0];
leg.length_eq = 0;
leg.torque = 0;
leg.spring_force = [0; 0];
leg.motor_force = [0; 0];
leg.gravity_force = [0; 0];
leg.ground_force = [0; 0];
leg.foot_force = [0; 0];


function body = makebody()
% Define fields of body struct
body.pos = [0; 0];
body.th = 0;
body.dpos = [0; 0];
body.dth = 0;
body.spring_a_force = [0; 0];
body.spring_b_force = [0; 0];
body.gravity_force = [0; 0];
body.ground_force = [0; 0];
body.force = [0; 0];
body.torque = 0;
