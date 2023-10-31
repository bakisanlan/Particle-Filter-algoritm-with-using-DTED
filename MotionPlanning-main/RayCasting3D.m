classdef RayCasting3D < handle
    %RAYCASTING3D Terrain scanning via 3D ray casting
    %   Performs 3D raycasting over an arbitrary generated terrain that is
    %   made of domes and blocks. Performs two types of sweeps, and arc and
    %   an returning a LiDAR point cloud like data.

    properties
        dr;
        dtheta;
        dphi;
        rayRange;
        positionLiDAR;    % x, y, z
        orientationLiDAR; % theta, phi
        mapWidth;
        realScanData;
        Zr;
        aTerrain;
        hFigure;
    end

    methods
        function obj = RayCasting3D(~,~)
            %RAYCASTING3D Construct an instance of this class
            %   Initialize scanner, terrain and map parameters.

            % Dimensions and scanner parameters
            obj.dr = 0.5;
            obj.rayRange = 1500;
            obj.positionLiDAR = [0,0,50];
            obj.orientationLiDAR = [0 45];
            obj.mapWidth = 100;
            obj.dtheta = 1;
            obj.dphi = 1;

            % Terrain
            obj.aTerrain = ArtificialTerrain;

            % Scan data
            obj.realScanData = {};
            obj.Zr = zeros(0,1);
        end

        function showMap(obj)
            %SHOWMAP Plot the map and the terrain.
            obj.hFigure = findobj('Type','figure','Tag','ContinuousTerrain');
            if isempty(obj.hFigure)
                obj.hFigure = figure;
                obj.hFigure.Tag = 'ContinuousTerrain';
            end
            figure(obj.hFigure); clf(obj.hFigure); hold on; legend;
            xlabel('x');ylabel('y');
            view(-25,10); pbaspect([1 1 1]); daspect([1 1 1]);

            [~,x,y,z] = obj.aTerrain.getDTED(10);
            surf(x,y,z,'DisplayName','Terrain','EdgeColor', [0.4660 0.6740 0.1880],'FaceColor','green','FaceLighting','gouraud','FaceAlpha',0.99);
            title('Terrain Scanning by LiDAR');

            axis([min(x,[],"all") max(x,[],"all") min(y,[],"all") max(y,[],"all") min(z,[],"all") obj.positionLiDAR(3)]);

            % Plot the LiDAR location
            plot3(obj.positionLiDAR(1),obj.positionLiDAR(2),obj.positionLiDAR(3),'k.','MarkerSize',20,'DisplayName','LiDAR Location')
        end

        function [x,y,z] = raycast(obj, theta, phi)
            %RAYCAST Shooting a ray, ray casting, at an angle theta from
            %the horizon and phi from the z-axis.
            %   The ray starts at the LiDAR sensor location and ends at the
            %   first hit with the terrain. It returns the x, y, z
            %   coordinates of the end point.
            r = 0;
            hit = false;
            x = nan;  y = nan; z = nan;
            while (~hit && r < obj.rayRange)
                r = r + obj.dr;
                ray_x = obj.positionLiDAR(1) + r*cos(theta/180*pi)*cos(phi/180*pi);
                ray_y = obj.positionLiDAR(2) + r*cos(theta/180*pi)*sin(phi/180*pi);
                ray_z = obj.positionLiDAR(3) + r*sin(theta/180*pi);
                zmap = obj.aTerrain.getTerrainElevation(ray_x,ray_y);
                if ray_z < zmap
                    hit = true;
                    x = ray_x;
                    y = ray_y;
                    z = zmap;
                end
            end
        end

        function [X,Y,Z] = sweep_line(obj, theta_0, theta_f, deltaPhi)
            %SWEEP_LINE Scans a line
            %   Scans a line starting at theta_0 and ending at theta_f.
            phi0 = obj.orientationLiDAR(2);
            phi = phi0 + deltaPhi;
            theta = theta_0:obj.dtheta:theta_f;
            n = numel(theta);
            X = zeros(n,1); Y = zeros(n,1); Z = zeros(n,1);
            for i = 1:n
                [x,y,z] = raycast(obj, theta(i),phi);
                X(i) = x; Y(i) = y;  Z(i) = z;
            end
        end

        function [X,Y,Z] = sweep_arc(obj, deltaPhi, theta)
            %SWEEP_ARC Scans a symmetric arc.
            %   Scans a symmetric arc around current phi provided by
            %   dPhi.
            phi0 = obj.orientationLiDAR(2);
            phi = (phi0-deltaPhi):obj.dphi:(phi0+deltaPhi);
            m = numel(phi);
            X = zeros(m,1); Y = zeros(m,1); Z = zeros(m,1);
            for j = 1:m
                [x,y,z] = raycast(obj, theta,phi(j));
                X(j) = x; Y(j) = y;  Z(j) = z;
            end
        end

        function scanRealTerrain(obj, flagPlot)
            %SCANREALTERRAIN Scan the terrain emulating a LiDAR
            %   This is supposed to be a LiDAR emulator, so ideally it
            %   includes uncertainities encountered by a real radar scan.

            if nargin == 1
                flagPlot = false;
            end

            [x1,y1,z1] = obj.sweep_line(-90,-30, 45);
            [x2,y2,z2] = obj.sweep_line(-90,-30, 0);
            [x3,y3,z3] = obj.sweep_line(-90,-30, -45);
            [x4,y4,z4] = obj.sweep_arc(170, -60);
            [x5,y5,z5] = obj.sweep_arc(170, -40);
            [x6,y6,z6] = obj.sweep_arc(170, -30);

            obj.realScanData = {{x1,y1,z1},{x2,y2,z2},...
                {x3,y3,z3},{x4,y4,z4},...
                {x5,y5,z5},{x6,y6,z6}};

            obj.Zr = [z1;z2;z3;z4;z5;z6];

            if flagPlot
                Xr = [x1;x2;x3;x4;x5;x6];
                Yr = [y1;y2;y3;y4;y5;y6];
                figure(obj.hFigure);
                plot3(Xr,Yr,obj.Zr,'r.','MarkerSize',10,'DisplayName','LiDAR Scans');
            end
        end
    end
end