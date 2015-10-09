function dY = bislip(t, Y)
% Y: [com_x; com_y; com_th; com_xdot; com_ydot; com_thdot; 
%     foot_x; foot_y; foot_xdot; foot_ydot; 
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

% Physical parameters
g = 9.81;
I_com = 1;
m_com = 10;
m_foot = 1;
k_leg = 1000;
k_ground = 1e5;
b_leg = 10;
b_ground = 10;
mu = 1;

% Environment
ground_x = [-1e3 1e3];
ground_y = [0 0];

% Control inputs
leg_a.length_eq = 1;
leg_b.length_eq = 1;
leg_a.torque = 0;
leg_b.torque = 0;

% Break Y out
com.pos = Y(1:2);
com.th = Y(3);
com.dpos = Y(4:5);
com.dth = Y(6);
leg_a.foot = Y(7:8);
leg_a.dfoot = Y(9:10);
leg_b.foot = Y(11:12);
leg_b.dfoot = Y(13:14);

    function legcalcs(leg)
        % Calculate lengths, derivatives, etc
        leg.vec = foot - com;
        leg.length = norm(leg.vec);
        if leg.length ~= 0
            leg.direction = leg.vec/leg.length;
        else
            leg.direction = [0; -1];
        end
        leg.angle = atan2(leg.direction(1), -leg.direction(2));
        
        leg.dvec = dfoot - dcom;
        leg.dlength = dot(leg.dvec, leg.direction);
        
        % Find how far the toe is pushed into the ground
        % Get first intersection of leg with ground, starting at center of mass
        [xi, yi, ii] = polyxpoly([com(1) leg.foot(1)], [com(2) leg.foot(2)], ...
            ground_x, ground_y);
        if length(xi) < 1
            leg.depth = 0;
            leg.ddepth = 0;
            leg.ground_normal = [0; 1];
        else
            depths = sqrt((xi - foot(1)).^2 + (yi - foot(2)).^2);
            [leg.depth, imax] = max(depths);
            leg.ddepth = dot(leg.dfoot, leg.direction);
            igs = ii(imax, 2);
            ground_tangent = [diff(xground(igs:igs+1)), diff(yground(igs:igs+1))];
            leg.ground_tangent = ground_tangent/norm(ground_tangent);
            leg.ground_normal = [-ground_tangent(2); ground_tangent(1)];
            if dot(-leg.direction, ground.normal) < 0
                leg.ground_tangent = -leg.ground_tangent;
                leg.ground_normal = -leg.ground_normal;
            end
        end
        
        % Forces
        % Compression positive
        leg.force = k_leg*(leg.length_eq - leg.length) - b_leg*leg.dlength;
        leg.ground_force = max(k_ground*leg.depth + b_ground*leg.ddepth, 0);
        friction_force = mu*leg.ground_force*dot(-leg.direction, leg.ground_normal);
        foot_net_force = leg.direction*leg.force - leg.direction*leg.ground_force;
        foot_net_force_tf = [dot(foot_net_force, leg.ground_tangent);
            dot(foot_net_force, leg.ground_normal)];
        friction_force = min(abs(foot_net_force_tf(1)), friction_force);
        leg.friction_force = -sign(foot_net_force_tf(1))*friction_force*leg.ground_tangent;
        leg.foot_force_vec = foot_net_force + friction_force;
    end

% Compute quantities for each leg
leg_a = legcalcs(leg_a);
leg_b = legcalcs(leg_b);



end

