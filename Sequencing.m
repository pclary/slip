classdef Sequencing < matlab.System & matlab.system.mixin.Propagates
    
    properties
        Ts = 1e-3;
        params = zeros(11, 1);
        filter_constant = 0.04;
    end
    
    
    properties (Access = private)
        forces_filtered;
        feet_contact_good;
        feet_down;
        feet_fade_latched;
        post_midstance;
        forces_filtered_max;
        beta_int_last;
    end
    
    
    properties (Nontunable)
        OutputBusName = 'sequencing_signal_bus';
    end
    
    
    methods (Access = protected)
        function setupImpl(~)
        end
        
        
        function signals = stepImpl(obj, legs)
            % Compute and filter leg forces
            forces = obj.params(4)*(legs([1 7]) - legs([3 9]));
            obj.forces_filtered = obj.forces_filtered*(1 - obj.filter_constant) ...
                                  + obj.filter_constant*forces;
            newmax = obj.forces_filtered > obj.forces_filtered_max;
            obj.forces_filtered_max(newmax) = obj.forces_filtered(newmax);
            
            % Apply thresholds
            weight = obj.params(1)*obj.params(11);
            
            obj.feet_contact_good(forces > weight) = true;
            obj.feet_contact_good(forces < weight/2) = false;
            
            feet_fade_rising = threshold_fade(obj.forces_filtered, weight/3, weight/8);
            feet_fade_falling = threshold_fade(obj.forces_filtered, weight/3, weight/8);
            feet_fade = feet_fade_rising;
            feet_fade(obj.feet_down) = feet_fade_falling(obj.feet_down);
            
            % Phase parameters
            [alpha, beta] = obj.phase(feet_fade, legs(5) - legs(11));
            
            % Detect events
            touchdown = ~obj.feet_fade_latched & feet_fade == 1;
            takeoff = obj.feet_fade_latched & feet_fade == 0;
            
            midstance = obj.feet_contact_good & ~obj.post_midstance & ...
                obj.forces_filtered < obj.forces_filtered_max;
            
            touchdown_fast = ~obj.feet_down & forces > weight/16;
            takeoff_fast = obj.feet_down & forces <= 0;
            
            % Apply events
            obj.feet_fade_latched(touchdown) = true;
            obj.feet_fade_latched(takeoff) = false;
            obj.feet_down(touchdown_fast) = true;
            obj.feet_down(takeoff_fast) = false;
            obj.post_midstance(midstance) = true;
            obj.post_midstance(takeoff_fast) = false;
            obj.forces_filtered_max(touchdown_fast) = 0;
            
            % Output bus
            signals.forces = forces;
            signals.forces_filtered = obj.forces_filtered;
            signals.feet_fade = feet_fade;
            signals.feet_contact_good = obj.feet_contact_good;
            signals.touchdown = touchdown;
            signals.touchdown_fast = touchdown_fast;
            signals.midstance = midstance;
            signals.takeoff = takeoff;
            signals.takeoff_fast = takeoff_fast;
            signals.phase = [alpha; beta];
        end
        
        
        function resetImpl(obj)
            obj.forces_filtered = [0; 0];
            obj.feet_contact_good = [false; false];
            obj.feet_down = [false; false];
            obj.feet_fade_latched = [false; false];
            obj.post_midstance = [false; false];
            obj.forces_filtered_max = [0; 0];
            obj.beta_int_last = 0;
        end
        
        
        function out = getOutputDataTypeImpl(obj)
            out = obj.OutputBusName;
        end
        
        
        function [flag_1] = isOutputFixedSizeImpl(~)
            flag_1 = true;
        end
        
        
        function [sz_1] = getOutputSizeImpl(~)
            sz_1 = [1 1];
        end
    end
    
    
    methods (Access = private)
        function [alpha, beta] = phase(obj, feet, th_diff)
            % Calculate phase parameters
            % alpha: bias towards leg A or leg B, [-1, 1]
            % betap: proportion total leg forces, depends on alpha, [0, 1]
            % beta: unwrapped betap, [0, 3)
            %   0 is flight, A to front
            %   1 is double support, A in front
            %   2 is flight, B to front
            %   3 is double support, B in front
            
            A = feet(1);
            B = feet(2);
            
            alpha = (A - B);
            if abs(alpha) ~= 1
                betap = (A + B - abs(alpha))/(2*(1 - abs(alpha)));
            else
                betap = NaN;
            end
            
            if ~isnan(betap)
                switch obj.beta_int_last
                    case 0
                        if th_diff >= 0
                            beta = betap;
                        else
                            beta = 4 - betap;
                        end
                    case 1
                        beta = 2 - betap;
                    case 2
                        if th_diff >= 0
                            beta = 2 - betap;
                        else
                            beta = 2 + betap;
                        end
                    case 3
                        beta = 4 - betap;
                    otherwise
                        % This should never happen
                        beta = NaN;
                end
                beta = mod(beta, 4);
            else
                if alpha >= 0
                    beta = 2;
                else
                    beta = 0;
                end
            end
            if beta - round(beta) < 1e2*eps(1)
                obj.beta_int_last = mod(round(beta), 4);
            end
        end
    end
end


function y = threshold_fade(u, high, low)
y = min(max((u - low)/(high - low), 0), 1);
end
