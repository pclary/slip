classdef BiSLIPGraphics < handle
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    properties (SetAccess=private)
        Fig = gobjects();
        Axes = gobjects();
        Body = gobjects();
        AngleA = gobjects();
        AngleB = gobjects();
        LengthA = gobjects();
        LengthB = gobjects();
        SpringA = gobjects();
        SpringB = gobjects();
        Ground = gobjects();
        BodyTrace = gobjects();
        VToeATrace = gobjects();
        VToeBTrace = gobjects();
        ToeATrace = gobjects();
        ToeBTrace = gobjects();
        DragLine = gobjects();
        DragIndicator = gobjects();
        ViewScale = 1;
        ViewCenter = [0; 0];
        ViewCenterOffset = [0; 0];
        PanEnabled = false;
        PanAnchor = [0; 0];
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Public Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    methods
        function obj = BiSLIPGraphics()
            obj.createGeometry();
            obj.reset();
        end
        
        
        function setState(obj, X, leg_targets)
            obj.addTracePoints(X, leg_targets);
            obj.updateTransforms(X);
        end
        
        
        function setGround(obj, ground_data)
            set(obj.Ground, 'XData', ground_data(:, 1), 'YData', ground_data(:, 2));
        end
        
        
        function reset(obj)
            obj.BodyTrace.clearpoints();
            obj.ToeATrace.clearpoints();
            obj.ToeBTrace.clearpoints();
            obj.VToeATrace.clearpoints();
            obj.VToeBTrace.clearpoints();
            obj.ViewScale = 1;
            obj.ViewCenter = [0; 0];
            obj.ViewCenterOffset = [0; 0];
            obj.PanEnabled = false;
            obj.PanAnchor = [0; 0];
            obj.disableDrag();
            set(obj.Ground, 'XData', [], 'YData', []);
        end
        
        
        function en = dragEnabled(obj)
            en = strcmp(obj.DragIndicator.Visible, 'on');
        end
        
        
        function out = isAlive(obj)
            out = obj.isvalid() && obj.Fig.isvalid() && ...
                obj.Axes.isvalid() && obj.Body.isvalid();
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access=private)
        function createGeometry(obj)
            fig = figure;
            obj.Fig = fig;
            ax = axes('Parent', fig);
            obj.Axes = ax;
            obj.Axes.DataAspectRatio = [1 1 1];
            obj.Axes.PlotBoxAspectRatio = [obj.Fig.Position(3:4) 1];
            obj.Axes.Position = [0 0 1 1];
            obj.Axes.XRuler.Visible = 'off';
            obj.Axes.YRuler.Visible = 'off';
            
            % Traces
            obj.VToeATrace = animatedline('Parent', ax, 'Color', 'blue', 'LineStyle', ':');
            obj.VToeBTrace = animatedline('Parent', ax, 'Color', 'red', 'LineStyle', ':');
            obj.BodyTrace = animatedline('Parent', ax, 'Color', 'green');
            obj.ToeATrace = animatedline('Parent', ax, 'Color', 'blue');
            obj.ToeBTrace = animatedline('Parent', ax, 'Color', 'red');
            
            % Ground
            obj.Ground = line('Parent', ax);
            obj.Ground.XData = [];
            obj.Ground.YData = [];
            
            obj.Body = hgtransform('Parent', ax);
            
            % Leg
            obj.AngleA = hgtransform('Parent', obj.Body);
            obj.AngleB = hgtransform('Parent', obj.Body);
            obj.LengthA = hgtransform('Parent', obj.AngleA);
            obj.LengthB = hgtransform('Parent', obj.AngleB);
            line([0 0], [0 -1], 'Parent', obj.LengthA);
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
            
            % Turn off hit test
            obj.BodyTrace.HitTest = 'off';
            obj.ToeATrace.HitTest = 'off';
            obj.ToeBTrace.HitTest = 'off';
            obj.AngleA.Children(1).HitTest = 'off';
            obj.AngleB.Children(1).HitTest = 'off';
            obj.DragLine.HitTest = 'off';
            obj.DragIndicator.HitTest = 'off';
            
            % Register callbacks
            obj.Axes.ButtonDownFcn = @obj.axesClick;
            body_rect.ButtonDownFcn = @obj.bodyClick;
            body_line.ButtonDownFcn = @obj.bodyClick;
            obj.Fig.SizeChangedFcn = @obj.setAxes;
            obj.Fig.WindowScrollWheelFcn = @obj.scrollWheel;
            obj.Fig.WindowButtonDownFcn = @obj.figMouseDown;
            obj.Fig.WindowButtonUpFcn = @obj.figMouseUp;
            obj.Fig.WindowButtonMotionFcn = @obj.mouseMove;
        end
        
        
        function updateTransforms(obj, X)
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            obj.Body.Matrix = makehgtform('translate', [X(1); X(3); 0])*makehgtform('zrotate', X(5));
            obj.AngleA.Matrix = makehgtform('zrotate', X(11));
            obj.AngleB.Matrix = makehgtform('zrotate', X(17));
            obj.LengthA.Matrix = makehgtform('scale', [1 max(X(9), 1e-3) 1]);
            obj.LengthB.Matrix = makehgtform('scale', [1 max(X(15), 1e-3) 1]);
            obj.SpringA.Matrix = springTransform(X(7), X(9));
            obj.SpringB.Matrix = springTransform(X(13), X(15));
            
            obj.setAxes();
            
            if obj.dragEnabled()
                obj.mouseMove();
            end
        end
        
        
        function addTracePoints(obj, X, leg_targets)
            obj.BodyTrace.addpoints(X(1), X(3));
            
            toe_a_x = X(1) + X(9)*sin(X(5) + X(11));
            toe_a_y = X(3) - X(9)*cos(X(5) + X(11));
            obj.ToeATrace.addpoints(toe_a_x, toe_a_y);
            
            toe_b_x = X(1) + X(15)*sin(X(5) + X(17));
            toe_b_y = X(3) - X(15)*cos(X(5) + X(17));
            obj.ToeBTrace.addpoints(toe_b_x, toe_b_y);
            
            vtoe_a_x = X(1) + leg_targets(1)*sin(X(5) + leg_targets(2));
            vtoe_a_y = X(3) - leg_targets(1)*cos(X(5) + leg_targets(2));
            obj.VToeATrace.addpoints(vtoe_a_x, vtoe_a_y);
            
            vtoe_b_x = X(1) + leg_targets(3)*sin(X(5) + leg_targets(4));
            vtoe_b_y = X(3) - leg_targets(3)*cos(X(5) + leg_targets(4));
            obj.VToeBTrace.addpoints(vtoe_b_x, vtoe_b_y);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Callbacks
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function axesClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.disableDrag();
            end
        end
        
        
        function bodyClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.toggleDrag();
            end
        end
        
        
        function figMouseDown(obj, ~, data)
            switch data.Source.SelectionType
                case 'extend' % MMB
                    obj.PanEnabled = true;
                    obj.PanAnchor = obj.Axes.CurrentPoint(1, 1:2)';
            end
        end
        
        
        function figMouseUp(obj, ~, data)
            switch data.Source.SelectionType
                case 'extend' % MMB
                    obj.PanEnabled = false;
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
            set(obj.DragLine, 'XData', [body(1) mouse(1)], 'YData', [body(2) mouse(2)]);
        end
        
        
        function setAxes(obj, ~, ~)
            if ~obj.dragEnabled()
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
        % Drag Functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function toggleDrag(obj)
            if obj.dragEnabled()
                obj.disableDragUnchecked();
            else
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
            obj.PanEnabled = false;
            obj.mouseMove();
        end
        
        
        function disableDragUnchecked(obj)
            vcdiff = obj.ViewCenter - obj.Body.Matrix(1:2, 4);
            obj.ViewCenterOffset = obj.ViewCenterOffset + vcdiff;
            obj.DragLine.Visible = 'off';
            obj.DragIndicator.Visible = 'off';
            obj.PanEnabled = false;
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
