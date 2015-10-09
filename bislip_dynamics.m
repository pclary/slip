function [dY, body, leg_a, leg_b] = bislip_dynamics(Y, u, params, ground_data)
% Y: [body_x; body_y; body_th; com_xdot; body_ydot; body_thdot; 
%     foot_a_x; foot_a_y; foot_a_xdot; foot_a_ydot; 
%     foot_b_x; foot_b_y; foot_b_xdot; foot_b_ydot]

% Physical parameters
body.mass =       params(1);
body.inertia =    params(2);
leg_a.mass =      params(3);
leg_b.mass =      params(3);
leg_a.stiffness = params(4);
leg_b.stiffness = params(4);
leg_a.damping =   params(5);
leg_b.damping =   params(5);
gravity =         params(6);

% Environment
ground.x =         ground_data(:, 1);
ground.y =         ground_data(:, 2);
ground.stiffness = ground_data(:, 3);
ground.damping =   ground_data(:, 4);
ground.friction =  ground_data(:, 5);

% Control inputs
leg_a.length_eq = u(1);
leg_b.length_eq = u(2);
leg_a.torque =    u(3);
leg_b.torque =    u(4);

% Break Y out
body.pos =    Y(1:2);
body.th =     Y(3);
body.dpos =   Y(4:5);
body.dth =    Y(6);
leg_a.foot =  Y(7:8);
leg_a.dfoot = Y(9:10);
leg_b.foot =  Y(11:12);
leg_b.dfoot = Y(13:14);

    function leg = legcalcs(leg)
        % Calculate lengths, derivatives, etc
        leg.vec = leg.foot - body.pos;
        leg.length = norm(leg.vec);
        if leg.length ~= 0
            leg.direction = leg.vec/leg.length;
        else
            leg.direction = [0; -1];
        end
        leg.angle = atan2(leg.direction(1), -leg.direction(2));
        
        leg.dvec = leg.dfoot - body.dpos;
        leg.dlength = dot(leg.dvec, leg.direction);
        
        % Find how far the toe is pushed into the ground
        % Get first intersection of leg with ground, starting at center of mass
        [xi, yi, ii] = polyxpoly([body.pos(1) leg.foot(1)], [body.pos(2) leg.foot(2)], ...
            ground.x, ground.y);
        if length(xi) < 1
            leg.depth = 0;
            leg.ddepth = 0;
            leg.ground_tangent = [1; 0];
            leg.ground_normal = [0; 1];
            leg.ground_stiffness = 0;
            leg.ground_damping = 0;
            leg.ground_friction = 0;
        else
            depths = sqrt((xi - leg.foot(1)).^2 + (yi - leg.foot(2)).^2);
            [leg.depth, imax] = max(depths);
            leg.ddepth = dot(leg.dfoot, leg.direction);
            igs = ii(imax, 2);
            ground_segment = [diff(ground.x(igs:igs+1)); diff(ground.y(igs:igs+1))];
            leg.ground_tangent = ground_segment/norm(ground_segment);
            leg.ground_normal = ccw90(leg.ground_tangent);
            if dot(-leg.direction, leg.ground_normal) < 0
                leg.ground_tangent = -leg.ground_tangent;
                leg.ground_normal = -leg.ground_normal;
            end
            intersection_vector = [xi(imax) - ground.x(igs); yi(imax) - ground.y(igs)];
            p = dot(intersection_vector, ground_segment)/norm(ground_segment)^2;
            leg.ground_stiffness = interpolate(ground.stiffness, igs, p);
            leg.ground_damping = interpolate(ground.stiffness, igs, p);
            leg.ground_friction = interpolate(ground.stiffness, igs, p);
        end
        
        % Forces
        % Compression positive
        leg.spring_force = leg.stiffness*(leg.length_eq - leg.length) - leg.damping*leg.dlength;
        if leg.length ~=0
            leg.motor_force = leg.torque/leg.length;
        else
            leg.motor_force = 0;
        end
        leg.ground_force = max(leg.ground_stiffness*leg.depth + ...
                               leg.ground_damping*leg.ddepth, 0);
        leg.gravity_force = leg.mass*gravity;
        
        % Add friction
        friction_force = leg.ground_friction*leg.ground_force*dot(-leg.direction, leg.ground_normal);
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

% Calculate forces on body
body.forces = -leg_a.spring_force*leg_a.direction ...
    - leg_b.spring_force*leg_b.direction ...
    + [0; -1]*gravity;
body.torque = -leg_a.torque + -leg_b.torque;

% Compose state derivative vector
dY = [body.dpos; body.dth; body.forces/body.mass; body.torque/body.inertia; 
      leg_a.dfoot; leg_a.foot_forces/leg_a.mass;
      leg_b.dfoot; leg_b.foot_forces/leg_b.mass];

end


function out = ccw90(v)
out = [-v(2); v(1)];
end


function out = interpolate(v, i, p)
out = v(i) + p*(v(i+1) - v(i));
end

