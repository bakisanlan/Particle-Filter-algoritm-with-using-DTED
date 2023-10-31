classdef ArtificialTerrain < handle
    %ARTIFICIALTERRAIN Defines the terrain for scanning and creating a digital
    %elevation model.
    %   Creates a terrain that has a continuous surface, and can also be
    %   used to create a digital elevation model.

    properties
        % Map dimensions
        mapWidth;

        % Dome parameters
        nd;
        xd;
        yd;
        rd;

        % Block parameters
        nb;
        xb;
        yb;
        sigma;
        zbh;

        % Ground
        ng;
        xg;
        yg;
        sigma_g;
        zgh;
    end

    methods
        function obj = ArtificialTerrain(~,~)
            %TERRAIN Construct a terrain
            %   Initialize terrain parameters.

            % Dimensions and scanner parameters
            obj.mapWidth = 1000;

            % Terrain topography generation parameters
            % Dome
            obj.nd = 10;
            obj.xd = obj.mapWidth*rand(1,obj.nd);
            obj.yd = obj.mapWidth*rand(1,obj.nd);
            obj.rd = 30 + 100*rand(1,obj.nd);

            % Blocks
            obj.nb = 20;
            obj.xb = obj.mapWidth*rand(1,obj.nb);
            obj.yb = obj.mapWidth*rand(1,obj.nb);
            obj.sigma = 10 + 100*rand(1,obj.nb);
            obj.zbh = 50 + 100*rand(1,obj.nb);

            % Structureless ground
            obj.ng = 6;
            obj.xg = 0:obj.mapWidth/(obj.ng-1):obj.mapWidth;
            obj.yg = 0:obj.mapWidth/(obj.ng-1):obj.mapWidth;
            obj.sigma_g = 10 + 1000*rand(1,obj.ng);
            obj.zgh = 0 + 50*randn(1,obj.ng);
        end

        function elevation = getTerrainElevation(obj,x,y)
            %getTerrainElevation Returns the elevation of the terrain.
            %   The elevation varies due to the features added into the map
            %   such as domes, and blocks.
            elevation = obj.dome(x,y) + obj.block(x,y) + obj.ground(x,y);
        end

        function [dted,x,y,z] = getDTED(obj, res)
            %createDTED Plot the map and the terrain.
            x = (-200:res:obj.mapWidth+200)';
            y = (-200:res:obj.mapWidth+200)';
            z = zeros(numel(x),numel(y));
            for i = 1:numel(x)
                for j = 1:numel(y)
                    z(j,i) = obj.getTerrainElevation(x(i),y(j));
                end
            end
            dted = [{x},{y},{z}];
        end

        function z = dome(obj,x, y)
            %DOME Creates a dome shape structure
            z = 0;
            if obj.nd == 0
                return
            end
            arg = obj.rd.^2 - ((x-obj.xd).^2 + (y-obj.yd).^2);
            arg(arg<0) = 0;  % Speeds up sqrt by eliminating imaginary output
            z = sum(sqrt(arg));
        end

        function z = block(obj,x, y)
            %BLOCK Creates a block shape structure
            z = 0;
            if obj.nb == 0
                return
            end
            z = sum(obj.zbh.*exp(- ( abs(x-obj.xb).^2.65 + abs(y-obj.yb).^2.65 ) ./(2*obj.sigma.^2)));
        end

        function z = ground(obj, x, y)
            %GROUND Structureless ground
            z = 0;
            if obj.ng == 0
                return
            end
            z = sum(obj.zgh.*exp(-((x-obj.xg).^2+(y-obj.yg).^2)./(2*obj.sigma_g.^2)));
        end

    end
end