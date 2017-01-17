classdef BipedVisualization < matlab.System & matlab.system.mixin.Propagates
    % Displays an interactive biped model.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    properties
        ground_data = zeros(2, 5);
        same_window = false;
    end
    
        
    properties (Access = private)
        fig
        ax
        Body
        angle_right
        angle_left
        length_right
        length_left
        spring_right
        spring_left
        ground
        ground_shading
        body_trace
        toe_trace_right
        toe_trace_left
        drag_line
        drag_indicator
        drag_pin_indicator
        drag_pinned = false;
        view_scale = 1;
        view_center = [0; 0];
        view_center_offset = [0; 0];
        pan_enabled = false;
        pan_anchor = [0; 0];
        fall_indicator
        state_evaluator
        X_last;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Matlab System Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = protected)
        
        function setupImpl(obj)
            obj.createGeometry();
            obj.state_evaluator = StateEvaluator();
        end
        
        
        function v = stepImpl(obj, X)
            obj.X_last = X;
            
            if ~obj.isAlive()
                v = [0; 0];
                return
            end
            
            obj.addTracePoints(X);
            obj.updateTransforms(X);
            drawnow;
            
            if obj.dragEnabled()
                X = obj.drag_line.XData;
                y = obj.drag_line.YData;
                v = [X(2) - X(1); y(2) - y(1)];
            else
                v = [0; 0];
            end
        end
        
        
        function resetImpl(obj)
            obj.body_trace.clearpoints();
            obj.toe_trace_right.clearpoints();
            obj.toe_trace_left.clearpoints();
            obj.view_scale = 1;
            obj.view_center = [0; 0];
            obj.view_center_offset = [0; 0];
            obj.pan_enabled = false;
            obj.pan_anchor = [0; 0];
            obj.disableDrag();
            obj.ground.XData = obj.ground_data(:, 1);
            obj.ground.YData = obj.ground_data(:, 2);
            obj.ground_shading.XData = obj.ground_data(:, 1);
            obj.ground_shading.YData = obj.ground_data(:, 2);
        end
        
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            [s.bt.x, s.bt.y] = obj.body_trace.getpoints();
            [s.ttr.x, s.ttr.y] = obj.toe_trace_right.getpoints();
            [s.ttl.x, s.ttl.y] = obj.toe_trace_left.getpoints();
            s.X = obj.X_last;
        end
        
        
        function loadObjectImpl(obj, s, wasLocked)
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            obj.setup(s.X);
            obj.body_trace.addpoints(s.bt.x, s.bt.y);
            obj.toe_trace_right.addpoints(s.ttr.x, s.ttr.y);
            obj.toe_trace_left.addpoints(s.ttl.x, s.ttl.y);
            obj.step(s.X);
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
    % Public Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        
        function out = isAlive(obj)
            out = obj.isvalid() && obj.fig.isvalid() && ...
                obj.ax.isvalid() && obj.Body.isvalid();
        end
        
        
        function out = getFig(obj)
            out = obj.fig;
        end
        
        
        function resetPartial(obj)
            obj.body_trace.clearpoints();
            obj.toe_trace_right.clearpoints();
            obj.toe_trace_left.clearpoints();
            obj.pan_enabled = false;
            obj.pan_anchor = [0; 0];
            obj.disableDrag();
            obj.ground.XData = obj.ground_data(:, 1);
            obj.ground.YData = obj.ground_data(:, 2);
            obj.ground_shading.XData = obj.ground_data(:, 1);
            obj.ground_shading.YData = obj.ground_data(:, 2);
        end
        
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods (Access = private)
        
        function en = dragEnabled(obj)
            en = strcmp(obj.drag_indicator.Visible, 'on');
        end
        
        
        function createGeometry(obj)
            persistent fig_persistent
            
            if obj.same_window
                if ~isa(fig_persistent, 'matlab.ui.Figure') || ~isvalid(fig_persistent)
                    fig_persistent = figure;
                end
                clf(fig_persistent);
                fig = fig_persistent;
            else
                fig = figure;
            end
            obj.fig = fig;
            
            ax = axes('Parent', fig);
            grid(ax, 'on');
            ax.GridColor = [1 1 1]*0.5;
            obj.ax = ax;
            obj.ax.DataAspectRatio = [1 1 1];
            obj.ax.PlotBoxAspectRatio = [obj.fig.Position(3:4) 1];
            obj.ax.Position = [0 0 1 1];
            obj.ax.XRuler.Visible = 'off';
            obj.ax.YRuler.Visible = 'off';
            
            % Traces
            obj.body_trace = animatedline('Parent', ax, 'Color', 'green', 'MaximumNumPoints', 90);
            obj.toe_trace_right = animatedline('Parent', ax, 'Color', 'blue', 'MaximumNumPoints', 90);
            obj.toe_trace_left = animatedline('Parent', ax, 'Color', 'red', 'MaximumNumPoints', 90);
            
            % ground
            obj.ground = line('Parent', ax);
            obj.ground.XData = obj.ground_data(:, 1);
            obj.ground.YData = obj.ground_data(:, 2);
            obj.ground_shading = patch('Parent', ax);
            obj.ground_shading.XData = obj.ground_data(:, 1);
            obj.ground_shading.YData = obj.ground_data(:, 2);
            obj.ground_shading.FaceAlpha = 0.1;
            obj.ground_shading.EdgeAlpha = 0;
            
            % Body frame
            obj.Body = hgtransform('Parent', ax);
            
            % Leg
            obj.angle_right = hgtransform('Parent', obj.Body);
            obj.angle_left = hgtransform('Parent', obj.Body);
            obj.length_right = hgtransform('Parent', obj.angle_right);
            obj.length_left = hgtransform('Parent', obj.angle_left);
            line([0 0], [0 -1], 'Parent', obj.length_right);
            line(0, -1, 'Marker', '.', 'MarkerFaceColor', 'Black', 'Parent', obj.length_right);
            line(0, -1, 'Marker', '.', 'MarkerFaceColor', 'Black', 'Parent', obj.length_left);
            line([0 0], [0 -1], 'Parent', obj.length_left);
            obj.spring_right = hgtransform('Parent', obj.angle_right);
            obj.spring_left = hgtransform('Parent', obj.angle_left);
            ncoils = 5;
            coilres = 16;
            spring_y = linspace(0, -1, (4*coilres)*ncoils+1);
            spring_x = sin(spring_y*2*pi*ncoils)*0.4;
            line(spring_x, spring_y, 'Parent', obj.spring_right);
            line(spring_x, spring_y, 'Parent', obj.spring_left);
            
            % Body
            body_rect = rectangle('Parent', obj.Body);
            body_radius = [0.2 0.2];
            body_rect.Position = [-0.5*body_radius(1) -0.5*body_radius(2) ...
                1*body_radius(1) 1*body_radius(2)];
            body_rect.Curvature = [1 1];
            body_rect.FaceColor = 'white';
            body_line = line(body_radius(1)*[0.2 0.5], [0 0], 'Parent', obj.Body);
            
            % Mouse-body line
            obj.drag_line = line('Parent', ax);
            obj.drag_line.Color = 'Magenta';
            obj.drag_line.LineStyle = '--';
            obj.drag_line.Visible = 'off';
            obj.drag_indicator = rectangle('Parent', obj.Body);
            obj.drag_indicator.EdgeColor = 'Magenta';
            obj.drag_indicator.FaceColor = 'Magenta';
            obj.drag_indicator.Curvature = [1 1];
            obj.drag_indicator.Position = [-0.01 -0.01 0.02 0.02];
            obj.drag_indicator.Visible = 'off';
            obj.drag_pin_indicator = line(0, 0, 'Parent', ax);
            obj.drag_pin_indicator.Visible = 'off';
            obj.drag_pin_indicator.Marker = 'x';
            obj.drag_pin_indicator.MarkerEdgeColor = 'Magenta';
            
            % Fall indicator
            obj.fall_indicator = uicontrol(obj.fig, 'Style', 'text');
            obj.fall_indicator.Position = [20 20 30 30];
            
            % Turn off hit test
            obj.body_trace.HitTest = 'off';
            obj.toe_trace_right.HitTest = 'off';
            obj.toe_trace_left.HitTest = 'off';
            obj.angle_right.Children(1).HitTest = 'off';
            obj.angle_left.Children(1).HitTest = 'off';
            obj.drag_line.HitTest = 'off';
            obj.drag_indicator.HitTest = 'off';
            obj.drag_pin_indicator.HitTest = 'off';
            
            % Register callbacks
            obj.ax.ButtonDownFcn = @obj.axClick;
            body_rect.ButtonDownFcn = @obj.axClick;
            body_line.ButtonDownFcn = @obj.axClick;
            obj.ground.ButtonDownFcn = @obj.axClick;
            obj.ground_shading.ButtonDownFcn = @obj.axClick;
            obj.fig.SizeChangedFcn = @obj.setax;
            obj.fig.WindowScrollWheelFcn = @obj.scrollWheel;
            obj.fig.WindowButtonDownFcn = @obj.figMouseDown;
            obj.fig.WindowButtonUpFcn = @obj.figMouseUp;
            obj.fig.WindowButtonMotionFcn = @obj.mouseMove;
        end
        
        
        function updateTransforms(obj, X)
            obj.Body.Matrix = makehgtform('translate', [X.body.x; X.body.y; 0]) * ...
                makehgtform('zrotate', X.body.theta);
            obj.angle_right.Matrix = makehgtform('zrotate', X.right.theta);
            obj.angle_left.Matrix = makehgtform('zrotate', X.left.theta);
            obj.length_right.Matrix = makehgtform('scale', [1 max(X.right.l, 1e-3) 1]);
            obj.length_left.Matrix = makehgtform('scale', [1 max(X.left.l, 1e-3) 1]);
            obj.spring_right.Matrix = springTransform(X.right.l_eq, X.right.l);
            obj.spring_left.Matrix = springTransform(X.left.l_eq, X.left.l);
            
            obj.setax();
            
            if obj.dragEnabled()
                obj.mouseMove();
            end
            
            % Update fall indicator
            v = obj.state_evaluator.stability(X);
            obj.fall_indicator.BackgroundColor = [sqrt(1 - v), sqrt(v), 0] * 0.7 + 0.3;
        end
        
        
        function addTracePoints(obj, X)
            obj.body_trace.addpoints(X.body.x, X.body.y);
            
            toe_a_x = X.body.x + X.right.l*sin(X.body.theta + X.right.theta);
            toe_a_y = X.body.y - X.right.l*cos(X.body.theta + X.right.theta);
            obj.toe_trace_right.addpoints(toe_a_x, toe_a_y);
            
            toe_b_x = X.body.x + X.left.l*sin(X.body.theta + X.left.theta);
            toe_b_y = X.body.y - X.left.l*cos(X.body.theta + X.left.theta);
            obj.toe_trace_left.addpoints(toe_b_x, toe_b_y);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Callbacks
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function axClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.toggleDrag();
                case 3 % RMB
                    obj.enableDrag();
                    obj.toggledrag_pinned();
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
            mouse = obj.ax.CurrentPoint(1, 1:2)';
            if obj.pan_enabled
                position_diff = mouse - obj.pan_anchor;
                obj.view_center_offset = obj.view_center_offset - position_diff;
                obj.setax();
            end
            body = obj.Body.Matrix(1:2, 4);
            if ~obj.drag_pinned
                set(obj.drag_line, 'XData', [body(1) mouse(1)], 'YData', [body(2) mouse(2)]);
            else
                x_pin = obj.drag_line.XData(2);
                y_pin = obj.drag_line.YData(2);
                set(obj.drag_line, 'XData', [body(1) x_pin], 'YData', [body(2) y_pin]);
            end
        end
        
        
        function setax(obj, ~, ~)
            if ~obj.dragEnabled() && ~obj.pan_enabled
                obj.view_center = obj.Body.Matrix(1:2, 4);
            end
            fr = obj.fig.Position(3)/obj.fig.Position(4);
            yw = obj.view_scale*1.2;
            xw = yw*fr;
            vc = obj.view_center + obj.view_center_offset;
            obj.ax.XLim = [vc(1) - xw, vc(1) + xw];
            obj.ax.YLim = [vc(2) - yw, vc(2) + yw];
            obj.ax.PlotBoxAspectRatio = [fr 1 1];
        end
        
        
        function scrollWheel(obj, ~, data)
            if data.VerticalScrollCount == 0
                % Sometimes gets called with VerticalScrollCount = 0
                % Ignore these
                return
            end
            sc = 2^(0.5*data.VerticalScrollCount);
            obj.view_scale = obj.view_scale*sc;
            
            % Keep ax.CurrentPoint constant during scaling
            mouse = obj.ax.CurrentPoint(1, 1:2)';
            vc = obj.view_center + obj.view_center_offset;
            offset = mouse - vc;
            obj.view_center_offset = mouse - offset*sc - obj.view_center;
                
            obj.setax();
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
            obj.drag_line.Visible = 'on';
            obj.drag_indicator.Visible = 'on';
            obj.drag_pin_indicator.Visible = 'off';
            obj.disablePan();
            obj.mouseMove();
        end
        
        
        function disableDragUnchecked(obj)
            vcdiff = obj.view_center - obj.Body.Matrix(1:2, 4);
            obj.view_center_offset = obj.view_center_offset + vcdiff;
            obj.drag_line.Visible = 'off';
            obj.drag_indicator.Visible = 'off';
            obj.disabledrag_pinned();
            obj.disablePan();
        end
        
        
        function enablePan(obj)
            if ~obj.pan_enabled
                obj.pan_enabled = true;
                obj.pan_anchor = obj.ax.CurrentPoint(1, 1:2)';
            end
        end
        
        
        function disablePan(obj)
            if obj.pan_enabled
                if ~obj.dragEnabled()
                    vcdiff = obj.view_center - obj.Body.Matrix(1:2, 4);
                    obj.view_center_offset = obj.view_center_offset + vcdiff;
                end
                obj.pan_enabled = false;
            end
        end
        
        function toggledrag_pinned(obj)
            if obj.drag_pinned
                obj.disabledrag_pinned();
            else
                obj.enabledrag_pinned();
            end
        end
        
        function disabledrag_pinned(obj)
            obj.drag_pinned = false;
            obj.drag_pin_indicator.Visible = 'off';
        end
        
        function enabledrag_pinned(obj)
            obj.drag_pinned = true;
            mouse = obj.ax.CurrentPoint(1, 1:2)';
            set(obj.drag_pin_indicator, 'XData', mouse(1), 'YData', mouse(2));
            obj.drag_pin_indicator.Visible = 'on';
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
