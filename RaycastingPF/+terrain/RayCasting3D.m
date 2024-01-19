classdef RayCasting3D < terrain.AbstractRayCasting3D
    %RAYCASTING3D Terrain scanning via 3D ray casting for a continuous
    %surface.
    %
    %   Performs 3D raycasting over a continuous arbitrary generated
    %   terrain that is made of domes and blocks. Performs two types of
    %   sweeps, and arc and an returning a LiDAR point cloud like data.
    %
    %   For more on references frames assumed, see the parent class
    %   documention.
    %
    %   See also AbstractRayCasting3D.

    properties
        dr;
        mapWidth;
        realScanData;
        Zr;
        aTerrain;
    end

    methods
        function obj = RayCasting3D(varargin)
            %RAYCASTING3D Construct an instance of this class
            %   Initialize scanner, terrain and map parameters.

            % Call parent class constructor
            obj = obj@terrain.AbstractRayCasting3D(varargin{:});

            % Dimensions and scanner parameters
            obj.dr = 0.5;


            % Terrain
            obj.aTerrain = terrain.ArtificialTerrain;

            % Scan data
            obj.realScanData = {};
            obj.Zr = zeros(0,1);
        end

        function showMap(obj)
            %SHOWMAP Plot the map and the terrain.
            [~,x,y,z] = obj.aTerrain.getDTED(10);

            obj.hFigure = findobj('Type','figure','Tag','ContinuousTerrain');
            if isempty(obj.hFigure)
                obj.hFigure = figure;
                obj.hFigure.Tag = 'ContinuousTerrain';
            end

            showMap@terrain.AbstractRayCasting3D(obj,x,y,z);

        end

        function [xs,ys,zs, xw, yw, zw] = raycast(obj, theta, psi_r)
            %RAYCAST Shooting a ray, ray casting, at an angle PSI from the
            %z-axis of ENU and THETA about the y-axis of the intermediate
            %frame.
            %
            %   The ray starts at the LiDAR sensor location and ends at the
            %   first hit with the terrain. It returns the x, y, z
            %   coordinates of the end point in the sensor frame. It also
            %   returns the same points in world frame for plotting
            %   purposes only.
            psi_s = obj.orientationLiDAR(1);
            r = 0;
            hit = false;
            xs = nan;  ys = nan; zs = nan; 

            % Returned only for plotting purposes
            xw = nan;  yw = nan; zw = nan;
            while (~hit && r < obj.rayRange)
                r = r + obj.dr;
                % Applying POSE to ray [1;0;0] in ray frame
                ray = terrain.RayCasting3D.wPOSEr(obj.positionLiDAR,psi_s,psi_r,theta) * [r;0;0;1];
                zmap = obj.aTerrain.getTerrainElevation(ray(1),ray(2));
                if ray(3) < zmap
                    hit = true;

                    % In world frame
                    xw = ray(1); yw = ray(2); zw = zmap;
                    
                    % In sensor frame
                    xyz_s = transpose(terrain.AbstractRayCasting3D.wRs(psi_s,true)) * ...
                        terrain.RayCasting3D.rTs(-obj.positionLiDAR) * [xw;yw;zw;1];
                    xs = xyz_s(1); ys =  xyz_s(2); zs =  xyz_s(3);
                end
            end
        end
    end
end