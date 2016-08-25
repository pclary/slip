classdef BipedVisualization < matlab.System & matlab.system.mixin.Propagates
    % Displays an interactive biped model.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties (Nontunable)
        env = struct();
        ground_data = zeros(1, 5)
    end
    
        
    properties (Access = private)
        Fig
        Axes
        Body
        AngleA
        AngleB
        LengthA
        LengthB
        SpringA
        SpringB
        Ground
        GroundShading
        BodyTrace
        ToeATrace
        ToeBTrace
        DragLine
        DragIndicator
        DragPinIndicator
        DragPinned = false;
        ViewScale = 1;
        ViewCenter = [0; 0];
        ViewCenterOffset = [0; 0];
        PanEnabled = false;
        PanAnchor = [0; 0];
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab System Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = protected)
        
        function setupImpl(obj)
            obj.createGeometry();
        end
        
        
        function v = stepImpl(obj, X)
            if ~obj.isAlive()
                v = [0; 0];
                return
            end
            
            obj.addTracePoints(X);
            obj.updateTransforms(X);
            drawnow;
            
            if obj.dragEnabled()
                X = obj.DragLine.XData;
                y = obj.DragLine.YData;
                v = [X(2) - X(1); y(2) - y(1)];
            else
                v = [0; 0];
            end
        end
        
        
        function resetImpl(obj)
            obj.BodyTrace.clearpoints();
            obj.ToeATrace.clearpoints();
            obj.ToeBTrace.clearpoints();
            obj.ViewScale = 1;
            obj.ViewCenter = [0; 0];
            obj.ViewCenterOffset = [0; 0];
            obj.PanEnabled = false;
            obj.PanAnchor = [0; 0];
            obj.disableDrag();
        end
        
        function out = getOutputSizeImpl(~)
            out = [2 1];
        end
        
        function out = getOutputDataTypeImpl(~)
            out = 'double';
        end
        
        function out = isOutputComplexImpl(~)
            out = false;
        end
        
        function out = isOutputFixedSizeImpl(~)
            out = true;
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = private)
        
        function en = dragEnabled(obj)
            en = strcmp(obj.DragIndicator.Visible, 'on');
        end
        
        
        function out = isAlive(obj)
            out = obj.isvalid() && obj.Fig.isvalid() && ...
                obj.Axes.isvalid() && obj.Body.isvalid();
        end
        
        
        function createGeometry(obj)
            fig = figure;
            obj.Fig = fig;
            ax = axes('Parent', fig);
            grid(ax, 'on');
            ax.GridColor = [1 1 1]*0.5;
            obj.Axes = ax;
            obj.Axes.DataAspectRatio = [1 1 1];
            obj.Axes.PlotBoxAspectRatio = [obj.Fig.Position(3:4) 1];
            obj.Axes.Position = [0 0 1 1];
            obj.Axes.XRuler.Visible = 'off';
            obj.Axes.YRuler.Visible = 'off';
            
            % Traces
            obj.BodyTrace = animatedline('Parent', ax, 'Color', 'green');
            obj.ToeATrace = animatedline('Parent', ax, 'Color', 'blue');
            obj.ToeBTrace = animatedline('Parent', ax, 'Color', 'red');
            
            % Ground
            obj.Ground = line('Parent', ax);
            obj.Ground.XData = obj.ground_data(:, 1);
            obj.Ground.YData = obj.ground_data(:, 2);
            obj.GroundShading = patch('Parent', ax);
            obj.GroundShading.XData = obj.ground_data(:, 1);
            obj.GroundShading.YData = obj.ground_data(:, 2);
            obj.GroundShading.FaceAlpha = 0.1;
            obj.GroundShading.EdgeAlpha = 0;
            
            % Body frame
            obj.Body = hgtransform('Parent', ax);
            
            % Leg
            obj.AngleA = hgtransform('Parent', obj.Body);
            obj.AngleB = hgtransform('Parent', obj.Body);
            obj.LengthA = hgtransform('Parent', obj.AngleA);
            obj.LengthB = hgtransform('Parent', obj.AngleB);
            line([0 0], [0 -1], 'Parent', obj.LengthA);
            line(0, -1, 'Marker', '.', 'MarkerFaceColor', 'Black', 'Parent', obj.LengthA);
            line(0, -1, 'Marker', '.', 'MarkerFaceColor', 'Black', 'Parent', obj.LengthB);
            line([0 0], [0 -1], 'Parent', obj.LengthB);
            obj.SpringA = hgtransform('Parent', obj.AngleA);
            obj.SpringB = hgtransform('Parent', obj.AngleB);
            ncoils = 5;
            coilres = 16;
            spring_y = linspace(0, -1, (4*coilres)*ncoils+1);
            spring_x = sin(spring_y*2*pi*ncoils)*0.4;
            line(spring_x, spring_y, 'Parent', obj.SpringA);
            line(spring_x, spring_y, 'Parent', obj.SpringB);
            
            % Body
            body_rect = rectangle('Parent', obj.Body);
            body_radius = [0.2 0.2];
            body_rect.Position = [-0.5*body_radius(1) -0.5*body_radius(2) ...
                1*body_radius(1) 1*body_radius(2)];
            body_rect.Curvature = [1 1];
            body_rect.FaceColor = 'white';
            body_line = line(body_radius(1)*[0.2 0.5], [0 0], 'Parent', obj.Body);
            
            % Mouse-body line
            obj.DragLine = line('Parent', ax);
            obj.DragLine.Color = 'Magenta';
            obj.DragLine.LineStyle = '--';
            obj.DragLine.Visible = 'off';
            obj.DragIndicator = rectangle('Parent', obj.Body);
            obj.DragIndicator.EdgeColor = 'Magenta';
            obj.DragIndicator.FaceColor = 'Magenta';
            obj.DragIndicator.Curvature = [1 1];
            obj.DragIndicator.Position = [-0.01 -0.01 0.02 0.02];
            obj.DragIndicator.Visible = 'off';
            obj.DragPinIndicator = line(0, 0, 'Parent', ax);
            obj.DragPinIndicator.Visible = 'off';
            obj.DragPinIndicator.Marker = 'x';
            obj.DragPinIndicator.MarkerEdgeColor = 'Magenta';
            
            % Turn off hit test
            obj.BodyTrace.HitTest = 'off';
            obj.ToeATrace.HitTest = 'off';
            obj.ToeBTrace.HitTest = 'off';
            obj.AngleA.Children(1).HitTest = 'off';
            obj.AngleB.Children(1).HitTest = 'off';
            obj.DragLine.HitTest = 'off';
            obj.DragIndicator.HitTest = 'off';
            obj.DragPinIndicator.HitTest = 'off';
            
            % Register callbacks
            obj.Axes.ButtonDownFcn = @obj.axesClick;
            body_rect.ButtonDownFcn = @obj.axesClick;
            body_line.ButtonDownFcn = @obj.axesClick;
            obj.Ground.ButtonDownFcn = @obj.axesClick;
            obj.GroundShading.ButtonDownFcn = @obj.axesClick;
            obj.Fig.SizeChangedFcn = @obj.setAxes;
            obj.Fig.WindowScrollWheelFcn = @obj.scrollWheel;
            obj.Fig.WindowButtonDownFcn = @obj.figMouseDown;
            obj.Fig.WindowButtonUpFcn = @obj.figMouseUp;
            obj.Fig.WindowButtonMotionFcn = @obj.mouseMove;
        end
        
        
        function updateTransforms(obj, X)
            obj.Body.Matrix = makehgtform('translate', [X.body.x; X.body.y; 0]) * ...
                makehgtform('zrotate', X.body.theta);
            obj.AngleA.Matrix = makehgtform('zrotate', X.right.theta);
            obj.AngleB.Matrix = makehgtform('zrotate', X.left.theta);
            obj.LengthA.Matrix = makehgtform('scale', [1 max(X.right.l, 1e-3) 1]);
            obj.LengthB.Matrix = makehgtform('scale', [1 max(X.left.l, 1e-3) 1]);
            obj.SpringA.Matrix = springTransform(X.right.l_eq, X.right.l);
            obj.SpringB.Matrix = springTransform(X.left.l_eq, X.left.l);
            
            obj.setAxes();
            
            if obj.dragEnabled()
                obj.mouseMove();
            end
        end
        
        
        function addTracePoints(obj, X)
            obj.BodyTrace.addpoints(X.body.x, X.body.y);
            
            toe_a_x = X.body.x + X.right.l*sin(X.body.theta + X.right.theta);
            toe_a_y = X.body.y - X.right.l*cos(X.body.theta + X.right.theta);
            obj.ToeATrace.addpoints(toe_a_x, toe_a_y);
            
            toe_b_x = X.body.x + X.left.l*sin(X.body.theta + X.left.theta);
            toe_b_y = X.body.y - X.left.l*cos(X.body.theta + X.left.theta);
            obj.ToeBTrace.addpoints(toe_b_x, toe_b_y);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Callbacks
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function axesClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.toggleDrag();
                case 3 % RMB
                    obj.enableDrag();
                    obj.toggleDragPinned();
            end
        end
        
        
        function figMouseDown(obj, ~, data)
            switch data.Source.SelectionType
                case 'extend' % MMB
                    obj.enablePan();
            end
        end
        
        
        function figMouseUp(obj, ~, data)
            switch data.Source.SelectionType
                case 'extend' % MMB
                    obj.disablePan();
            end
        end
        
        
        function mouseMove(obj, ~, ~)
            mouse = obj.Axes.CurrentPoint(1, 1:2)';
            if obj.PanEnabled
                position_diff = mouse - obj.PanAnchor;
                obj.ViewCenterOffset = obj.ViewCenterOffset - position_diff;
                obj.setAxes();
            end
            body = obj.Body.Matrix(1:2, 4);
            if ~obj.DragPinned
                set(obj.DragLine, 'XData', [body(1) mouse(1)], 'YData', [body(2) mouse(2)]);
            else
                x_pin = obj.DragLine.XData(2);
                y_pin = obj.DragLine.YData(2);
                set(obj.DragLine, 'XData', [body(1) x_pin], 'YData', [body(2) y_pin]);
            end
        end
        
        
        function setAxes(obj, ~, ~)
            if ~obj.dragEnabled() && ~obj.PanEnabled
                obj.ViewCenter = obj.Body.Matrix(1:2, 4);
            end
            fr = obj.Fig.Position(3)/obj.Fig.Position(4);
            yw = obj.ViewScale*1.2;
            xw = yw*fr;
            vc = obj.ViewCenter + obj.ViewCenterOffset;
            obj.Axes.XLim = [vc(1) - xw, vc(1) + xw];
            obj.Axes.YLim = [vc(2) - yw, vc(2) + yw];
            obj.Axes.PlotBoxAspectRatio = [fr 1 1];
        end
        
        
        function scrollWheel(obj, ~, data)
            if data.VerticalScrollCount == 0
                % Sometimes gets called with VerticalScrollCount = 0
                % Ignore these
                return
            end
            sc = 2^(0.5*data.VerticalScrollCount);
            obj.ViewScale = obj.ViewScale*sc;
            
            % Keep Axes.CurrentPoint constant during scaling
            mouse = obj.Axes.CurrentPoint(1, 1:2)';
            vc = obj.ViewCenter + obj.ViewCenterOffset;
            offset = mouse - vc;
            obj.ViewCenterOffset = mouse - offset*sc - obj.ViewCenter;
                
            obj.setAxes();
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Drag/Pan Functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function toggleDrag(obj)
            if obj.dragEnabled()
                obj.disableDragUnchecked();
            else
                obj.enableDragUnchecked();
            end
        end
        
        
        function enableDrag(obj)
            if ~obj.dragEnabled()
                obj.enableDragUnchecked();
            end
        end
        
        
        function disableDrag(obj)
            if obj.dragEnabled()
                obj.disableDragUnchecked();
            end
        end
        
        
        function enableDragUnchecked(obj)
            obj.DragLine.Visible = 'on';
            obj.DragIndicator.Visible = 'on';
            obj.DragPinIndicator.Visible = 'off';
            obj.disablePan();
            obj.mouseMove();
        end
        
        
        function disableDragUnchecked(obj)
            vcdiff = obj.ViewCenter - obj.Body.Matrix(1:2, 4);
            obj.ViewCenterOffset = obj.ViewCenterOffset + vcdiff;
            obj.DragLine.Visible = 'off';
            obj.DragIndicator.Visible = 'off';
            obj.disableDragPinned();
            obj.disablePan();
        end
        
        
        function enablePan(obj)
            if ~obj.PanEnabled
                obj.PanEnabled = true;
                obj.PanAnchor = obj.Axes.CurrentPoint(1, 1:2)';
            end
        end
        
        
        function disablePan(obj)
            if obj.PanEnabled
                if ~obj.dragEnabled()
                    vcdiff = obj.ViewCenter - obj.Body.Matrix(1:2, 4);
                    obj.ViewCenterOffset = obj.ViewCenterOffset + vcdiff;
                end
                obj.PanEnabled = false;
            end
        end
        
        function toggleDragPinned(obj)
            if obj.DragPinned
                obj.disableDragPinned();
            else
                obj.enableDragPinned();
            end
        end
        
        function disableDragPinned(obj)
            obj.DragPinned = false;
            obj.DragPinIndicator.Visible = 'off';
        end
        
        function enableDragPinned(obj)
            obj.DragPinned = true;
            mouse = obj.Axes.CurrentPoint(1, 1:2)';
            set(obj.DragPinIndicator, 'XData', mouse(1), 'YData', mouse(2));
            obj.DragPinIndicator.Visible = 'on';
        end
    end
    
end


function T = springTransform(leq, l)

ncoils = 5;

leq_spring = 0.5;
weq_spring = 0.1;
l_spring = leq_spring + l - leq;
w_spring = real(sqrt(leq_spring^2 - l_spring^2 + (5*weq_spring*2*pi)^2)/ncoils/2/pi);

end_offset = 0.05;
spring_translation = l - l_spring - end_offset;

l_spring = max(l_spring, 1e-3);
w_spring = max(w_spring, 1e-3);

T = makehgtform('translate', [0 -spring_translation 0])*makehgtform('scale', [w_spring l_spring 1]);

end
