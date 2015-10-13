function [body, leg_a, leg_b] = bislip_kinematics(Y)
% body: [x; y; xdot; ydot; th; thdot]
% leg: [x; y; xdot; ydot; th; thdot; l; ldot; xdir; ydir]

% Break Y out
body = Y(1:6);
foot_a = Y(7:10);
foot_b = Y(11:14);

leg_a = leg_kinematics(foot_a, body);
leg_b = leg_kinematics(foot_b, body);


function leg = leg_kinematics(foot, body)
% Calculate lengths, derivatives, etc
leg = zeros(10, 1);
leg(1:4) = foot; % position, velocity
vec = foot(1:2) - body(1:2);
dvec = foot(3:4) - body(3:4);
leg(7) = sqrt(vec(1)^2 + vec(2)^2); % length
if leg(7) ~= 0 % length ~= 0
    leg(9:10) = vec/leg(7); % direction
    leg(6) = (-leg(10)*dvec(1) + leg(9)*dvec(2))/leg(7); % thdot
else
    leg(9:10) = [0; -1]; % direction
    leg(6) = 0; % thdot
end
leg(5) = atan2(leg(9), -leg(10)); % th
leg(8) = dvec(1)*leg(9) + dvec(2)*leg(10); % lengthdot
