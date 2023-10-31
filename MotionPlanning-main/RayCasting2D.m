classdef RayCasting2D
    %RAYCASTING2D 2D ray casting
    %   Performs 2D raycasting

    properties
        x0;
        sigma0;
        dr;
        rayRange;
        posLiDAR;
        mapWidth;
    end

    methods
        function obj = RayCasting2D(~,~)
            %RayCasting Construct an instance of this class
            %   Detailed explanation goes here

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
            obj.showMap

            plot(obj.posLiDAR(1),obj.posLiDAR(2),'r.','MarkerSize',10,'DisplayName','LiDAR Location')
        end

        function elevation = terrain(obj,x)
            %map Summary of this method goes here
            %   Detailed explanation goes here
            elevation = sum(exp(-(x-obj.x0).^2./(2*obj.sigma0.^2)));
        end

        function showMap(obj)
            %showMap Summary of this method goes here
            %   Detailed explanation goes here
            x = (0:0.01:obj.mapWidth)';
            y = zeros(numel(x),1);
            for i = 1:length(x)
                y(i) = obj.terrain(x(i));
            end
            plot(x,y,'g-','DisplayName','Terrain');
            title('Terrain Scanning by LiDAR');
        end

        function [x,y] = ray(obj, theta)
            %ray Summary of this method goes here
            %   Detailed explanation goes here
            r = 0;
            hit = false;
            x = nan;  y = nan;
            while (~hit && r < obj.rayRange)
                r = r + obj.dr;
                ray_x = obj.posLiDAR(1) + r*cos(theta/180*pi);
                ray_y = obj.posLiDAR(2) + r*sin(theta/180*pi);
                ymap = obj.terrain(ray_x);
                if ray_y < ymap
                    hit = true;
                    x = ray_x;
                    y = ymap;
                end
            end
        end


        function [X,Y] = scan(obj, theta_0, theta_f)
            %scan Summary of this method goes here
            %   Detailed explanation goes here

            theta = theta_0:0.1:theta_f;
            n = numel(theta);
            X = zeros(n,1); Y = zeros(n,1);
            for i = 1:n
                [x,y] = ray(obj, theta(i));
                X(i) = x; Y(i) = y;
            end
            plot(X,Y,'b.','DisplayName','LiDAR Scans');

            % Remove NaN data due to out of range issues
            idx = or(isnan(X), isnan(Y));
            X(idx) = []; Y(idx) = [];
            figure(2); plot(X,Y,'.');
            title('LiDAR Point Cloud');
        end


    end
end