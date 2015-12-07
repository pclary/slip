classdef BiSLIPGraphics < handle
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Private Properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    properties (SetAccess=private)
        Fig = gobjects();
        Axes = gobjects();
        Body = gobjects();
        LegA = gobjects();
        LegB = gobjects();
        Ground = gobjects();
        BodyTrace = gobjects();
        ToeATrace = gobjects();
        ToeBTrace = gobjects();
        MouseLine = gobjects();
        ClickIndicator = gobjects();
        BodyRadius = [0.2; 0.2];
        SpringWidth = 0.1;
        Scale = 1;
        Center = [0; 0];
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Public Methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    methods
        function obj = BiSLIPGraphics()
            obj.createGeometry();
            obj.reset();
        end
        
        
        function setState(obj, X)
            obj.addTracePoints(X);
            obj.updateTransforms(X);
        end
        
        
        function setGround(obj, ground_data)
            set(obj.Ground, 'XData', ground_data(:, 1), 'YData', ground_data(:, 2));
        end
        
        
        function reset(obj)
            obj.BodyTrace.clearpoints();
            obj.ToeATrace.clearpoints();
            obj.ToeBTrace.clearpoints();
            obj.Scale = 1;
            obj.disableDrag();
            set(obj.Ground, 'XData', [], 'YData', []);
        end
        
        
        function en = dragEnabled(obj)
            en = strcmp(obj.ClickIndicator.Visible, 'on');
        end
        
        
        function out = isAlive(obj)
            out = obj.isvalid() && obj.Fig.isvalid() && obj.Axes.isvalid();
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
            b = rectangle('Parent', obj.Body);
            b.Position = [-0.5*obj.BodyRadius(1) -0.5*obj.BodyRadius(2) ...
                1*obj.BodyRadius(1) 1*obj.BodyRadius(2)];
            b.Curvature = [1 1];
            b.FaceColor = 'white';
            line(obj.BodyRadius(1)*[0.2 0.5], [0 0], 'Parent', obj.Body);
            
            % Mouse-body line
            obj.MouseLine = line('Parent', ax);
            obj.MouseLine.Color = 'red';
            obj.MouseLine.LineStyle = '--';
            obj.MouseLine.Visible = 'off';
            obj.ClickIndicator = rectangle('Parent', obj.Body);
            obj.ClickIndicator.EdgeColor = [1 0 0];
            obj.ClickIndicator.FaceColor = [1 0 0];
            obj.ClickIndicator.Curvature = [1 1];
            obj.ClickIndicator.Position = [-0.01 -0.01 0.02 0.02];
            obj.ClickIndicator.Visible = 'off';
            
            % Turn off hit test
            obj.BodyTrace.HitTest = 'off';
            obj.ToeATrace.HitTest = 'off';
            obj.ToeBTrace.HitTest = 'off';
            obj.LegA.Children(1).HitTest = 'off';
            obj.LegB.Children(1).HitTest = 'off';
            obj.MouseLine.HitTest = 'off';
            obj.ClickIndicator.HitTest = 'off';
            
            % Register callbacks
            obj.Axes.ButtonDownFcn = @obj.axesClick;
            obj.Body.Children(1).ButtonDownFcn = @obj.bodyClick;
            obj.Body.Children(3).ButtonDownFcn = @obj.bodyClick;
            obj.Fig.SizeChangedFcn = @obj.setAxes;
            obj.Fig.WindowScrollWheelFcn = @obj.scrollWheel;
        end
        
        
        function updateTransforms(obj, X)
            % X: [body_x;    body_xdot;    body_y;  body_ydot;  body_th;  body_thdot;
            %     leg_a_leq; leg_a_leqdot; leg_a_l; leg_a_ldot; leg_a_th; leg_a_thdot;
            %     leg_b_leq; leg_b_leqdot; leg_b_l; leg_b_ldot; leg_b_th; leg_b_thdot]
            obj.Body.Matrix = makehgtform('translate', [X(1); X(3); 0])*makehgtform('zrotate', X(5));
            obj.LegA.Matrix = makehgtform('zrotate', X(11))*makehgtform('scale', [1 X(9) 1]);
            obj.LegB.Matrix = makehgtform('zrotate', X(17))*makehgtform('scale', [1 X(15) 1]);
            
            obj.setAxes();
            
            if obj.dragEnabled()
                obj.mouseMove();
            end
        end
        
        
        function addTracePoints(obj, X)
            obj.BodyTrace.addpoints(X(1), X(3));
            
            toe_a_x = X(1) + X(9)*sin(X(5) + X(11));
            toe_a_y = X(3) - X(9)*cos(X(5) + X(11));
            obj.ToeATrace.addpoints(toe_a_x, toe_a_y);
            
            toe_b_x = X(1) + X(15)*sin(X(5) + X(17));
            toe_b_y = X(3) - X(15)*cos(X(5) + X(17));
            obj.ToeBTrace.addpoints(toe_b_x, toe_b_y);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Callbacks
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function axesClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.disableDrag();
                case 2 % MMB
                    
                case 3 % RMB
                    
            end
        end
        
        
        function bodyClick(obj, ~, data)
            switch data.Button
                case 1 % LMB
                    obj.toggleDrag();
                case 2 % MMB
                    
                case 3 % RMB
                    
            end
        end
        
        
        function mouseMove(obj, ~, ~)
            mxy = obj.Axes.CurrentPoint(1, 1:2);
            bxy = obj.Body.Matrix(1:2, 4);
            set(obj.MouseLine, 'XData', [bxy(1) mxy(1)], 'YData', [bxy(2) mxy(2)]);
        end
        
        
        function setAxes(obj, ~, ~)
            if ~obj.dragEnabled()
                obj.Center = obj.Body.Matrix(1:2, 4);
            end
            fr = obj.Fig.Position(3)/obj.Fig.Position(4);
            yw = obj.Scale;
            xw = yw*fr;
            obj.Axes.XLim = [obj.Center(1) - xw, obj.Center(1) + xw];
            obj.Axes.YLim = [obj.Center(2) - yw, obj.Center(2) + yw];
            obj.Axes.PlotBoxAspectRatio = [fr 1 1];
        end
        
        
        function scrollWheel(obj, ~, data)
            sc = 2^(0.5*data.VerticalScrollCount);
            obj.Scale = obj.Scale*sc;
            if obj.dragEnabled()
                % Keep Axes.CurrentPoint constant during scaling
                mouse = obj.Axes.CurrentPoint(1, 1:2)';
                offset = mouse - obj.Center;
                obj.Center = mouse - offset*sc;
            end
            obj.setAxes();
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Drag Functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function toggleDrag(obj)
            if obj.dragEnabled()
                obj.disableDrag();
            else
                obj.enableDrag();
            end
        end
        
        
        function enableDrag(obj)
            obj.MouseLine.Visible = 'on';
            obj.ClickIndicator.Visible = 'on';
            obj.Fig.WindowButtonMotionFcn = @obj.mouseMove;
            obj.mouseMove();
        end
        
        
        function disableDrag(obj)
            obj.MouseLine.Visible = 'off';
            obj.ClickIndicator.Visible = 'off';
            obj.Fig.WindowButtonMotionFcn = '';
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Pan Functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function enablePan(obj)
            
        end
        
        
        function disablePan(obj)
            
        end
    end
    
end
