classdef StateEvaluator < handle
   
    properties (Access = private)
        weights
    end
    
    
    methods
        
        function obj = StateEvaluator()
            w = coder.load('state_eval_weights.mat');
            obj.weights = w;
        end
        
        
        function [v, vs, vg] = value(obj, X, terrain, goal)
            vs = obj.stability(X, terrain);
            vg = obj.goal_value(X, goal);
            v = obj.combine_value(vs, vg);
        end
        
        
        function vs = stability(obj, X, terrain)
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
            vs = exp(out(1)) / sum(exp(out));
            
            % Absolute crash check
            if X.body.y < 0 || abs(X.body.theta) > pi/2 || abs(X.right.theta - X.left.theta) > pi*0.8
                vs = 0;
            end
        end
        
        
        function vg = goal_value(~, X, goal)
            % Percent-based velocity error score
            saturation = 2; % Full error
            deadband = 1.1; % Less than this (multiplicative) difference treated as zero error
            em = max(abs(log(min(max(X.body.dx / goal.dx, 1/saturation), saturation))) - log(deadband), 0) / log(saturation / deadband);
            
            % Difference-based velocity error score
            saturation = 1;
            deadband = 0.1;
            ed = min(max(abs(X.body.dx - goal.dx) - deadband, 0) / (saturation - deadband), 1);
            
            % Error is the minimum of the two error scores, and score is inverted
            vg = 1 - min(em, ed);
        end
        
        
        function v = combine_value(~, vs, vg)
            if vs < 0.5
                v = vs;
            else
                v = (vg + 1) / 2;
            end
        end
        
    end
    
end
