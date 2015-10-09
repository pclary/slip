function dY = bislip(t, Y)
% Y: [com_x; com_y; com_th; com_xdot; com_ydot; com_thdot; 
%     foot_x; foot_y; foot_xdot; foot_ydot; 
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

% Physical parameters
gravity = 9.81;
m_foot = 1;
k_leg = 1000;
b_leg = 10;

com.mass = 10;
com.inertia = 1;
leg_a.mass = m_foot;
leg_b.mass = m_foot;
leg_a.stiffness = k_leg;
leg_b.stiffness = k_leg;
leg_a.damping = b_leg;
leg_b.damping = b_leg;

% Environment
ground.x = [-1e3 1e3];
ground.y = [0 0];
ground.mu = 1;
ground.stiffness = 1e5;
ground.damping = 10;

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

    function leg = legcalcs(leg)
        % Calculate lengths, derivatives, etc
        leg.vec = leg.foot - com.pos;
        leg.length = norm(leg.vec);
        if leg.length ~= 0
            leg.direction = leg.vec/leg.length;
        else
            leg.direction = [0; -1];
        end
        leg.angle = atan2(leg.direction(1), -leg.direction(2));
        
        leg.dvec = leg.dfoot - com.dpos;
        leg.dlength = dot(leg.dvec, leg.direction);
        
        % Find how far the toe is pushed into the ground
        % Get first intersection of leg with ground, starting at center of mass
        [xi, yi, ii] = polyxpoly([com.pos(1) leg.foot(1)], [com.pos(2) leg.foot(2)], ...
            ground.x, ground.y);
        if length(xi) < 1
            leg.depth = 0;
            leg.ddepth = 0;
            leg.ground_tangent = [1; 0];
            leg.ground_normal = [0; 1];
        else
            depths = sqrt((xi - leg.foot(1)).^2 + (yi - leg.foot(2)).^2);
            [leg.depth, imax] = max(depths);
            leg.ddepth = dot(leg.dfoot, leg.direction);
            igs = ii(imax, 2);
            ground_tangent = [diff(ground.x(igs:igs+1)); diff(ground.y(igs:igs+1))];
            leg.ground_tangent = ground_tangent/norm(ground_tangent);
            leg.ground_normal = ccw90(leg.ground_tangent);
            if dot(-leg.direction, leg.ground_normal) < 0
                leg.ground_tangent = -leg.ground_tangent;
                leg.ground_normal = -leg.ground_normal;
            end
        end
        
        % Forces
        % Compression positive
        leg.spring_force = leg.stiffness*(leg.length_eq - leg.length) - leg.damping*leg.dlength;
        if leg.length ~=0
            leg.motor_force = leg.torque/leg.length;
        else
            leg.motor_force = 0;
        end
        leg.ground_force = max(ground.stiffness*leg.depth + ground.damping*leg.ddepth, 0);
        leg.gravity_force = leg.mass*gravity;
        
        % Add friction
        friction_force = ground.mu*leg.ground_force*dot(-leg.direction, leg.ground_normal);
        foot_net_forces = leg.spring_force*leg.direction ...
            + leg.ground_force*(-leg.direction) ...
            + leg.motor_force*ccw90(leg.direction) ...
            + leg.gravity_force*[0; -1];
        foot_net_force_tf = [dot(foot_net_forces, leg.ground_tangent);
            dot(foot_net_forces, leg.ground_normal)];
        friction_force = min(abs(foot_net_force_tf(1)), friction_force);
        leg.friction_forces = -sign(foot_net_force_tf(1))*friction_force*leg.ground_tangent;
        leg.foot_forces = foot_net_forces + leg.friction_forces;
    end

% Compute quantities for each leg
leg_a = legcalcs(leg_a);
leg_b = legcalcs(leg_b);

com.forces = -leg_a.spring_force*leg_a.direction ...
    - leg_b.spring_force*leg_b.direction ...
    + [0; -1]*gravity;
com.torque = -leg_a.torque + -leg_b.torque;

dY = [com.dpos; com.dth; com.forces/com.mass; com.torque/com.inertia; 
    leg_a.dfoot; leg_a.foot_forces/leg_a.mass;
    leg_b.dfoot; leg_b.foot_forces/leg_b.mass];

end


function out = ccw90(v)
out = [-v(2); v(1)];
end

