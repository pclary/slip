classdef BiSLIPGraphics < handle

    properties (SetAccess=private)
        Axes = gobjects();
        Body = gobjects();
        BodyVis = gobjects();
        LegA = gobjects();
        LegB = gobjects();
        Ground = gobjects();
        BodyTrace = gobjects();
        ToeATrace = gobjects();
        ToeBTrace = gobjects();
        StepsA = gobjects();
        StepsB = gobjects();
        BodyPos = [0; 1];
        BodyAngle = 0;
        ToeAPos = [0; 0];
        ToeBPos = [0; 0];
        BodyRadius = [0.2; 0.2];
        SpringWidth = 0.1;
    end
    
    methods
        function obj = BiSLIPGraphics()
            obj.createGeometry();
            obj.updateTransforms();
            obj.clearTrace();
        end
        function setState(obj, body, bodyth, toeA, toeB)
            obj.BodyPos = body(:);
            obj.BodyAngle = bodyth;
            obj.ToeAPos = toeA(:);
            obj.ToeBPos = toeB(:);
            obj.addTracePoints();
            obj.updateTransforms();
        end
        function clearTrace(obj)
            obj.BodyTrace.clearpoints();
            obj.ToeATrace.clearpoints();
            obj.ToeBTrace.clearpoints();
            obj.StepsA.XData = [];
            obj.StepsA.YData = [];
            obj.StepsB.XData = [];
            obj.StepsB.YData = [];
        end
        function r = isAlive(obj)
            r = isvalid(obj) && isvalid(obj.Ground) && ...
                isvalid(obj.Body) && isvalid(obj.LegA) && ...
                isvalid(obj.LegB) && isvalid(obj.BodyTrace) && ...
                isvalid(obj.ToeATrace) && isvalid(obj.ToeBTrace) && ...
                isvalid(obj.StepsA) && isvalid(obj.StepsB);
        end
        function setGround(obj, groundfun, numpts)
            [xb, ~] = obj.BodyTrace.getpoints();
            [xta, ~] = obj.ToeATrace.getpoints();
            [xtb, ~] = obj.ToeBTrace.getpoints();
            minx = min([xb(:); xta(:); xtb(:)]);
            maxx = max([xb(:); xta(:); xtb(:)]);
            width = maxx - minx;
            xg = linspace(minx - width*0.1, maxx + width*0.1, numpts);
            yg = groundfun(xg);
            set(obj.Ground, 'XData', xg, 'YData', yg);
        end
        function setSteps(obj, xstepsA, ystepsA, xstepsB, ystepsB)
            obj.StepsA.XData = xstepsA;
            obj.StepsA.YData = ystepsA;
            obj.StepsB.XData = xstepsB;
            obj.StepsB.YData = ystepsB;
        end
        function addStepA(obj, step)
            obj.StepsA.XData = [obj.Steps.XData, step(1)];
            obj.StepsA.YData = [obj.Steps.YData, step(2)];
        end
        function addStepB(obj, step)
            obj.StepsB.XData = [obj.Steps.XData, step(1)];
            obj.StepsB.YData = [obj.Steps.YData, step(2)];
        end
    end
    
    methods (Access=private)
        function createGeometry(obj)
            fig = figure;
            ax = axes('Parent', fig);
            obj.Axes = ax;
            axis(ax, 'equal');
            title(ax, 'BiSLIP');
            xlabel(ax, 'x (m)');
            ylabel(ax, 'y (m)');
            
            % Traces
            obj.BodyTrace = animatedline('Parent', ax, 'Color', 'green');
            obj.ToeATrace = animatedline('Parent', ax, 'Color', 'blue');
            obj.ToeBTrace = animatedline('Parent', ax, 'Color', 'red');
            
            % Ground
            obj.Ground = line('Parent', ax);
            obj.Ground.XData = [];
            obj.Ground.YData = [];
            
            obj.Body = hgtransform('Parent', ax);
            obj.LegA = hgtransform('Parent', obj.Body);
            obj.LegB = hgtransform('Parent', obj.Body);
            obj.BodyVis = hgtransform('Parent', obj.Body);
            
            % Leg
            l1 = 0.25;
            l2 = 0.15;
            coils = 5;
            hpitch = (1-l1-l2)/coils/2;
            xs =  [0, 0, repmat([1  -1], 1, coils), 0, 0]*obj.SpringWidth/2;
            ys = -[0, l1, (l1+hpitch/2):hpitch:(1-l2-hpitch/2), 1-l2, 1];
            line(xs, ys, 'Parent', obj.LegA);
            line(xs, ys, 'Parent', obj.LegB);
            
            % Body
            b = rectangle('Parent', obj.BodyVis);
            b.Position = [-0.5*obj.BodyRadius(1) -0.5*obj.BodyRadius(2) ...
                1*obj.BodyRadius(1) 1*obj.BodyRadius(2)];
            b.Curvature = [1 1];
            b.FaceColor = 'white';
            line(obj.BodyRadius(1)*[0.1 0.5], [0 0], 'Parent', obj.BodyVis);
            
            % Step points
            obj.StepsA = line('Parent', ax');
            obj.StepsA.LineStyle = 'none';
            obj.StepsA.Marker = 'o';
            obj.StepsA.Color = 'cyan';
            obj.StepsA.XData = [];
            obj.StepsA.YData = [];
            obj.StepsB = line('Parent', ax');
            obj.StepsB.LineStyle = 'none';
            obj.StepsB.Marker = 'o';
            obj.StepsB.Color = 'magenta';
            obj.StepsB.XData = [];
            obj.StepsB.YData = [];
        end
        function updateTransforms(obj)
            obj.Body.Matrix = makehgtform('translate', [obj.BodyPos; 0]);
            obj.BodyVis.Matrix = makehgtform('zrotate', obj.BodyAngle);
            legA = obj.BodyPos - obj.ToeAPos;
            thetaA = atan2(-legA(1), legA(2));
            lengthA = norm(legA);
            if ~isnan(thetaA)
                obj.LegA.Matrix = makehgtform('zrotate', thetaA)*makehgtform('scale', [1 lengthA 1]);
            end
            legB = obj.BodyPos - obj.ToeBPos;
            thetaB = atan2(-legB(1), legB(2));
            lengthB = norm(legB);
            if ~isnan(thetaB)
                obj.LegB.Matrix = makehgtform('zrotate', thetaB)*makehgtform('scale', [1 lengthB 1]);
            end
        end
        function addTracePoints(obj)
            obj.BodyTrace.addpoints(obj.BodyPos(1), obj.BodyPos(2));
            obj.ToeATrace.addpoints(obj.ToeAPos(1), obj.ToeAPos(2));
            obj.ToeBTrace.addpoints(obj.ToeBPos(1), obj.ToeBPos(2));
        end
    end
    
end
