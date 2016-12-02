classdef StateEvaluator < handle
   
    properties (Access = private)
        weights
    end
    
    
    methods
        
        function obj = StateEvaluator()
            w = load('state_eval_weights.mat');
            obj.weights = w;
        end
        
        
        function v = value(obj, X, goal, terrain)
            ps = obj.stability(X);
            pg = obj.goal(X, goal);
            v = min([ps, pg]);
        end
        
        
        function ps = stability(obj, X)
            % Construct input vector
            X.body.theta = mod(X.body.theta + pi, 2*pi) - pi;
            input = [...
                X.body.theta;
                X.body.dx;
                X.body.dy;
                X.body.dtheta;
                X.right.l;
                X.right.l_eq;
                X.right.theta;
                X.right.theta_eq;
                X.right.dl;
                X.right.dl_eq;
                X.right.dtheta;
                X.right.dtheta_eq;
                X.left.l;
                X.left.l_eq;
                X.left.theta;
                X.left.theta_eq;
                X.left.dl;
                X.left.dl_eq;
                X.left.dtheta;
                X.left.dtheta_eq];
            
            % Hidden layers
            h0 = max(obj.weights.b0 + obj.weights.w0*input, 0);
            h1 = max(obj.weights.b1 + obj.weights.w1*h0, 0);
            
            % Output layer
            out = obj.weights.b2 + obj.weights.w2*h1;
            
            % Softmax
            ps = exp(out(1)) / sum(exp(out));
            
            % Absolute crash check
            if X.body.y < 0 || abs(X.body.theta) > pi/2 || abs(X.right.theta - X.left.theta) > pi*0.8
                ps = 0;
            end
            
            if ps > 0.5
                ps = 1;
            end
        end
        
        
        function pg = goal(~, X, goal)
            % Percent-based velocity error score
            saturation = 2; % Less than this (multiplicative) difference treated as zero error
            deadband = 1.1; % Full error
            em = max(abs(log(min(max(X.body.dx / goal.dx, 1/saturation), saturation))) - log(deadband), 0) * log(saturation);
            
            % Difference-based velocity error score
            saturation = 1;
            deadband = 0.1;
            ed = min(max(abs(X.body.dx - goal.dx) - deadband, 0) / (saturation - deadband), 1);
            
            % Error is the minimum of the two error scores, and score is inverted
            pg = 1 - min(em, ed);
            
            % Scale score to have a minimum of 0.5 so velocity error doesn't
            % take precedence over stability
            pg = pg/2 + 0.5;
        end
        
    end
    
end


function out = clamp(x, l, h)
out = min(max(x, l), h);
end
