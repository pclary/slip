classdef Environment
    
    properties
        ground_data = zeros(2, 5);
    end
    
    
    methods
        function terrain = getLocalTerrain(obj, x)
            terrain = Terrain();
            
            % Snap center to a multiple of the x stride
            npts = numel(terrain.height);
            xdiff = (terrain.xend - terrain.xstart) / npts;
            x = round(x / xdiff) * xdiff;
            
            % Get the height samples
            terrain.xstart = terrain.xstart + x;
            terrain.xend = terrain.xend + x;
            terrain.height = interp1(obj.ground_data(:, 1), obj.ground_data(:, 2), ...
                linspace(terrain.xstart, terrain.xend, npts)');
            
            % Set other parameters using the center value
            terrain.stiffness = interp1(obj.ground_data(:, 1), obj.ground_data(:, 3), x);
            terrain.damping = interp1(obj.ground_data(:, 1), obj.ground_data(:, 4), x);
            terrain.friction = interp1(obj.ground_data(:, 1), obj.ground_data(:, 5), x);
        end
    end
    
end
