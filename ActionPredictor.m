classdef ActionPredictor < handle
   
    properties (Access = private)
        w
    end
    
    
    methods
        
        function obj = ActionPredictor()
            w = coder.load('action_pred_weights.mat');
            obj.w = w;
        end
        
        
        function out = predict(obj, X, cstate, terrain, goal)
            % Create state vector
            state = single([...
                mod(X.body.theta + pi, 2*pi) - pi;
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
                X.left.dtheta_eq;
                cstate.right.phase;
                cstate.right.foot_x_last;
                cstate.right.foot_x_target;
                cstate.left.phase;
                cstate.left.foot_x_last;
                cstate.left.foot_x_target;
                cstate.body_ddx;
                cstate.body_dx_last]);
            
            % Preprocess terrain
            height = single(max(terrain.height, -2));
            
            % Convolutional terrain input
            lconv = [];
            for i = 1:numel(obj.w.lconv_b)
                lconv(:, i) = obj.w.lconv_b(i) + conv(height, squeeze(obj.w.lconv_w(i, 1, :)), 'valid');
            end
            lconv = max(lconv, 0);
            
            % Add state vector
            lcat = [state; lconv(:)];
            
            % Dense layers
            ld1 = max(obj.w.ld1_b + obj.w.ld1_w * lcat, 0);
            ld2 = max(obj.w.ld2_b + obj.w.ld2_w * ld1, 0);
            ld3 = max(obj.w.ld3_b + obj.w.ld3_w * ld2, 0);
            
            % Output
            [~, out] = sort(obj.w.ld4_b + obj.w.ld4_w * ld3, 'descend');
        end
        
    end
    
end
