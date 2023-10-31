classdef RayCasting3DMesh < handle
    %RAYCASTING3DMESH Terrain scanning via 3D ray casting for a meshed
    %surface.
    %   Performs 3D raycasting over an arbitrary generated terrain that is
    %   made of domes and blocks. Triangle meshes are used to mesht the
    %   surface. Performs two types of sweeps, and arc and an returning a
    %   LiDAR point cloud like data.

    properties
        dtheta;
        dphi;
        rayRange;
        positionLiDAR;    % x, y, z
        orientationLiDAR; % theta, phi
        hFigure;
        DTED;
        x;
        y;
        z;
    end

    methods
        function obj = RayCasting3DMesh(DTED)
            %RAYCASTING3D Construct an instance of this class
            %   Initialize scanner, terrain and map parameters.

            % Dimensions and scanner parameters
            obj.rayRange = 1000;
            obj.positionLiDAR = [0;0;50];
            obj.orientationLiDAR = [0 45];
            obj.dtheta = 1;
            obj.dphi = 1;

            % Scan data
            obj.DTED = DTED;
            obj.x = obj.DTED{1};
            obj.y = obj.DTED{2};
            obj.z = obj.DTED{3};
        end

        function showMap(obj)
            %SHOWMAP Plot the map and the terrain.
            obj.hFigure = findobj('Type','figure','Tag','MeshedTerrain');
            if isempty(obj.hFigure)
                obj.hFigure = figure;
                obj.hFigure.Tag = 'MeshedTerrain';
            end
            figure(obj.hFigure);clf(obj.hFigure); hold on; legend;
            xlabel('x');ylabel('y');
            view(-25,10); pbaspect([1 1 1]); daspect([1 1 1]);

            surf(obj.x,obj.y,obj.z,'DisplayName','Terrain','EdgeColor', [0.4660 0.6740 0.1880],'FaceColor','green','FaceLighting','gouraud','FaceAlpha',0.90);
            title('Terrain Scanning by LiDAR');

            axis([min(obj.x,[],"all") max(obj.x,[],"all") min(obj.y,[],"all") max(obj.y,[],"all") min(obj.z,[],"all") obj.positionLiDAR(3)]);

            % Plot the LiDAR location
            plot3(obj.positionLiDAR(1),obj.positionLiDAR(2),obj.positionLiDAR(3),'k.','MarkerSize',20,'DisplayName','LiDAR Location')
        end

        function [Px,Py,Pz] = raycast(obj, theta, phi)
            %RAYCAST Shooting a ray, ray casting, at an angle theta from
            %the horizon and phi from the z-axis.
            %   The ray starts at the LiDAR sensor location and ends at the 
            %   first hit with the terrain. It returns the x, y, z
            %   coordinates of the ray-triangle intersection point.
            %   Line Segment-to-Triangle Mesh Intersection: This is based
            %   on Tomas Möller & Ben Trumbore (1997) Fast, Minimum Storage
            %   Ray-Triangle Intersection, Journal of Graphics Tools, 2:1,
            %   21-28.    https://doi.org/10.1080/10867651.1997.10487468

            [Px,Py,Pz] = deal(nan);

            % r = r0 + D;
            r0 =  obj.positionLiDAR;
            D = [cos(theta/180*pi)*cos(phi/180*pi);
                 cos(theta/180*pi)*sin(phi/180*pi);
                 sin(theta/180*pi)];
            
            ni = numel(obj.x);
            nj = numel(obj.y);
            for j = 1:(nj-1)
                if abs(obj.y(j) - r0(2)) > obj.rayRange, continue; end
                for i = 1:(ni-1)
                    if abs(obj.x(i) - r0(1)) > obj.rayRange, continue; end
                    i0  = i;  i1 = i+1; j0 = j;  j1 = j+1;
                    V0  = [obj.x(i0); obj.y(j0); obj.z(j0,i0)];
                    V1  = [obj.x(i1); obj.y(j0); obj.z(j0,i1)];
                    V2  = [obj.x(i1); obj.y(j1); obj.z(j1,i1)];
                    V3  = [obj.x(i0); obj.y(j1); obj.z(j1,i0)];

                    for half = 1:2
                        if half == 1
                            E1 = V1 - V0;
                            E2 = V2 - V0;
                        else
                            E1 = V2 - V0;
                            E2 = V3 - V0;
                        end
                        CommonDenominator = det([-D E1 E2]);

                        if CommonDenominator == 0
                            return
                        elseif CommonDenominator > 0
                            t = det([r0-V0 E1 E2]) / CommonDenominator;
                            u = det([-D r0-V0 E2 ])  / CommonDenominator;
                            v = det([-D E1 r0-V0])  / CommonDenominator;
                            if (u >= 0) && (v >= 0) && (u+v <= 1) && (t>0)
                                if t > obj.rayRange, return; end
                                Px = V0(1) + u*E1(1) + v*E2(1);
                                Py = V0(2) + u*E1(2) + v*E2(2);
                                Pz = V0(3) + u*E1(3) + v*E2(3);
                                return;
                            end
                        end
                    end
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
                [Px,Py,Pz] = raycast(obj, theta(i),phi);
                X(i) = Px; Y(i) = Py;  Z(i) = Pz;
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
                [Px,Py,Pz] = raycast(obj, theta,phi(j));
                X(j) = Px; Y(j) = Py;  Z(j) = Pz;
            end
        end

        function [X,Y,Z] = scanDigitalTerrain(obj, flagPlot)
            %SCANDIGITALTERRAIN Scan a digital terrain database
            %   This is supposed to scan a prior map.

            if nargin == 1
                flagPlot = false;
            end

            [x1,y1,z1] = obj.sweep_line(-90,-30, 45);
            [x2,y2,z2] = obj.sweep_line(-90,-30, 0);
            [x3,y3,z3] = obj.sweep_line(-90,-30, -45);
            [x4,y4,z4] = obj.sweep_arc(170, -60);
            [x5,y5,z5] = obj.sweep_arc(170, -40);
            [x6,y6,z6] = obj.sweep_arc(170, -30);

            X = [x1;x2;x3;x4;x5;x6];
            Y = [y1;y2;y3;y4;y5;y6];
            Z = [z1;z2;z3;z4;z5;z6];

            if flagPlot
                figure(obj.hFigure);
                plot3(X,Y,Z,'r.','MarkerSize',10,'DisplayName','LiDAR Scans');
            end
        end
    end
end