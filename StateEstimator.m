classdef StateEstimator < matlab.System
    
    properties
        Ts = 1e-3;
        params = zeros(11, 1);
    end
    
    properties (Access = private)
        x_last;
        xdot_last;
        y_last;
        ydot_last;
        feet_last;
        x_td_last;
        slope;
        slopes;
    end
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        function [X, slope] = stepImpl(obj, orientation, legs, signals)
            if ~any(signals.feet_contact_good)
                % Use ballistic trajectory, adding leg forces
                m = obj.params(1);
                g = obj.params(11);
                xddot = -sum(signals.forces.*sin(orientation(1) + legs([5 11])))/m;
                yddot =  sum(signals.forces.*cos(orientation(1) + legs([5 11])))/m - g;
                xdot = obj.xdot_last + xddot*obj.Ts;
                ydot = obj.ydot_last + yddot*obj.Ts;
                x = obj.x_last + xdot*obj.Ts;
                y = obj.y_last + ydot*obj.Ts;
            else
                % Estimate body state using leg kinematics
                if all(signals.feet_contact_good)
                    [xdot_a, y_a] = leg_calculations(orientation, legs(1:6));
                    [xdot_b, y_b] = leg_calculations(orientation, legs(7:12));
                    y = (y_a + y_b)/2;
                    xdot = (xdot_a + xdot_b)/2;
                elseif signals.feet_contact_good(1)
                    [xdot, y] = leg_calculations(orientation, legs(1:6));
                else % signals.feet_contact_good(2)
                    [xdot, y] = leg_calculations(orientation, legs(7:12));
                end
                
                x = obj.x_last + xdot*obj.Ts;
                
                if any(obj.feet_last)
                    ydot = (y - obj.y_last)/obj.Ts;
                    
                    % LPF on ydot when on the ground
                    freq = 100; %hz
                    a = 2*pi*freq*obj.Ts/(2*pi*freq*obj.Ts + 1);
                    ydot = ydot*a + obj.ydot_last*(1 - a);
                else % touchdown event, can't compute ydot
                    ydot = obj.ydot_last;
                    
                    if signals.feet_contact_good(1)
                        x_td = obj.x_last + legs(3)*sin(orientation(1) + legs(5));
                        y_td = obj.y_last - legs(3)*cos(orientation(1) + legs(5));
                    else
                        x_td = obj.x_last + legs(9)*sin(orientation(1) + legs(11));
                        y_td = obj.y_last - legs(9)*cos(orientation(1) + legs(11));
                    end
                    slope_new = y_td/(x_td - obj.x_td_last);
                    obj.x_td_last = x_td;
                    obj.slopes = [obj.slopes(2:end); slope_new];
                    obj.slope = median(obj.slopes);
                end
            end
            
            % Slope adjustment
            y_adj = y - obj.slope*(x - obj.x_td_last);
            
            obj.x_last = x;
            obj.xdot_last = xdot;
            obj.y_last = y;
            obj.ydot_last = ydot;
            obj.feet_last = signals.feet_contact_good;
            
            X = [x; xdot; y_adj; ydot; orientation; legs];
            X(5) = mod(X(5) + pi, 2*pi) - pi;
            slope = obj.slope;
        end
        
        function resetImpl(obj)
            obj.x_last = 0;
            obj.xdot_last = 0;
            obj.y_last = 1;
            obj.ydot_last = 0;
            obj.feet_last = [0; 0];
            obj.x_td_last = 0;
            obj.slope = 0;
            obj.slopes = [0; 0; 0];
        end
    end
end


function [xdot, y] = leg_calculations(orientation, leg)
% Estimate height above ground and forward speed using leg kinematics

l = leg(3);
ldot = leg(4);
th = orientation(1) + leg(5);
thdot = orientation(2) + leg(6);

xdot = -ldot*sin(th) - l*thdot*cos(th);
y = l*cos(th);
end
