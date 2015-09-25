classdef SlipGraphics < handle

    properties (SetAccess=private)
        Body = gobjects();
        Leg = gobjects();
        Ground = gobjects();
        BodyTrace = gobjects();
        ToeTrace = gobjects();
        BodyPos = [0; 1];
        ToePos = [0; 0];
        BodyRadius = 0.2;
        SpringWidth = 0.2;
    end
    
    methods
        function obj = SlipGraphics()
            obj.createGeometry();
            obj.updateTransforms();
        end
        function setState(obj, body, toe)
            obj.BodyPos = body(:);
            obj.ToePos = toe(:);
            obj.addTracePoints();
            obj.updateTransforms();
        end
        function clearTrace(obj)
            obj.BodyTrace.clearpoints();
            obj.ToeTrace.clearpoints();
        end
        function r = isAlive(obj)
            r = isvalid(obj) && isvalid(obj.Ground) && ...
                isvalid(obj.Body) && isvalid(obj.Leg) && ...
                isvalid(obj.BodyTrace) && isvalid(obj.ToeTrace);
        end
        function setGround(obj, groundfun, numpts)
            [xb, ~] = obj.BodyTrace.getpoints();
            [xt, ~] = obj.ToeTrace.getpoints();
            minx = min(min(xb), min(xt));
            maxx = max(max(xb), max(xt));
            width = maxx - minx;
            obj.Ground.XData = linspace(minx - width*0.1, ...
                maxx + width*0.1, numpts);
            obj.Ground.YData = groundfun(obj.Ground.XData);
        end
    end
    
    methods (Access=private)
        function createGeometry(obj)
            fig = figure;
            ax = axes('Parent', fig);
            ax.DataAspectRatioMode = 'manual';
            ax.DataAspectRatio = [1 1 1];
            
            % Traces
            obj.BodyTrace = animatedline('Parent', ax, 'Color', 'red');
            obj.ToeTrace = animatedline('Parent', ax, 'Color', 'blue');
            
            obj.Body = hgtransform('Parent', ax);
            obj.Leg = hgtransform('Parent', obj.Body);
            
            % Leg
            xs =  [0 0    1  -1   1  -1   1  -1   0    0]*obj.SpringWidth/2;
            ys = -[0 0.25 0.3 0.4 0.5 0.6 0.7 0.8 0.85 1];
            l = line(xs, ys, 'Parent', obj.Leg);
            
            % Body
            b = rectangle('Parent', obj.Body);
            b.Position = obj.BodyRadius*[-0.5 -0.5 1 1];
            b.Curvature = [1 1];
            b.FaceColor = 'white';
            
            % Ground
            obj.Ground = line('Parent', ax);
        end
        function updateTransforms(obj)
            obj.Body.Matrix = makehgtform('translate', [obj.BodyPos; 0]);
            leg = obj.BodyPos - obj.ToePos;
            theta = atan2(-leg(1), leg(2));
            length = norm(leg);
            obj.Leg.Matrix = makehgtform('zrotate', theta)*makehgtform('scale', [1 length 1]);
        end
        function addTracePoints(obj)
            obj.BodyTrace.addpoints(obj.BodyPos(1), obj.BodyPos(2));
            obj.ToeTrace.addpoints(obj.ToePos(1), obj.ToePos(2));
        end
    end
    
end
