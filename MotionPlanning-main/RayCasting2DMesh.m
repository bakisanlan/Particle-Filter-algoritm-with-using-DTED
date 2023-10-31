classdef RayCasting2DMesh < handle
    %RAYCASTING2DMESH 2D ray casting for a meshed surface.
    %   Performs 2D raycasting for a meshed surface using line segments.

    properties
        x0;
        sigma0;
        dr;
        rayRange;
        posLiDAR;
        mapWidth;
        DTED;
    end

    methods
        function obj = RayCasting2DMesh(~,~)
            %RayCasting2DMesh Construct an instance of this class.
            %   Constructor that sets default map and terrain dimensions,
            %   scanner parameters, and displays the map.

            % Dimensions and scanner parameters
            obj.dr = 0.1;
            obj.rayRange = 100;
            obj.posLiDAR = [0,10];
            obj.mapWidth = 100;

            hF1 = figure(1); clf(hF1); hold on; legend;
            axis([0 obj.mapWidth 0 obj.posLiDAR(2)]);

            % Terrain topography generation parameters
            n = 6;
            obj.x0 = 0:obj.mapWidth/(n-1):obj.mapWidth;
            obj.sigma0 = 1 + 100*rand(1,n);
            obj.DTED = {};
            obj.showMap;

            plot(obj.posLiDAR(1),obj.posLiDAR(2),'r.','MarkerSize',10,'DisplayName','LiDAR Location')
        end

        function elevation = getTerrainElevation(obj,x)
            %getTerrainElevation Returns elevation at the specified
            %position.
            %   Returns elevation value used to create DTED.
            elevation = sum(exp(-(x-obj.x0).^2./(2*obj.sigma0.^2)));
        end

        function showMap(obj)
            %showMap Summary of this method goes here
            %   Detailed explanation goes here
            x = (0:10:obj.mapWidth)';
            y = zeros(numel(x),1);
            for i = 1:length(x)
                y(i) = obj.getTerrainElevation(x(i));
            end
            obj.DTED = num2cell([x,y]);
            plot(x,y,'g-','DisplayName','Terrain');
            title('Terrain Scanning by LiDAR');
        end

        function [x,y] = ray(obj, theta)
            %ray Casting a ray and returning intersection point.
            %   Casting a ray and test for intersection with all surface
            %   mesh elements.
            x = nan;  y = nan;

            rx0 = obj.posLiDAR(1); rx = rx0 + cos(theta/180*pi);
            ry0 = obj.posLiDAR(2); ry = ry0 + sin(theta/180*pi);
            n = size(obj.DTED,1);
            for i = 2:n
                [xl, yl] = obj.DTED{i-1,:};
                [xu, yu] = obj.DTED{i,:};

                % Line Segment-to-Line Segment Intersection
                % This is based on Antonio, Franklin. "Faster line segment
                % intersection." Graphics Gems III (IBM Version). Morgan
                % Kaufmann, 1992. 199-202.
                % https://doi.org/10.1016/B978-0-08-050755-2.50045-2

                CommonDenominator = (rx0-rx)*(yl-yu)-(ry0-ry)*(xl-xu);
                if CommonDenominator == 0
                    return
                end
                s =  ((rx0-xl)*(yl-yu)-(ry0-yl)*(xl-xu))/CommonDenominator;
                t =  ((rx0-xl)*(ry0-ry)-(ry0-yl)*(rx0-rx))/CommonDenominator;

                if and(and(t >= 0, t <= 1), s > 0)
                    x = xl + t*(xu - xl);
                    y = yl + t*(yu - yl);
                end
            end
        end

        function [X,Y] = scan(obj, theta_0, theta_f)
            %scan Scans the terrains.
            %   Scans the terrain by ray casting in a number of directions
            %   by varying the theta angle within and initial and a final
            %   theta.

            theta = theta_0:0.1:theta_f;
            n = numel(theta);
            X = zeros(n,1); Y = zeros(n,1);
            for i = 1:n
                [x,y] = ray(obj, theta(i));
                X(i) = x; Y(i) = y;
            end
            plot(X,Y,'b.','DisplayName','LiDAR Scans');
        end

    end
end