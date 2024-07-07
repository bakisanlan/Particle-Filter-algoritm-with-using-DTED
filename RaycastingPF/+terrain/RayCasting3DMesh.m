classdef RayCasting3DMesh < terrain.AbstractRayCasting3D
    %RAYCASTING3DMESH Terrain scanning via 3D ray casting for a meshed
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
        DTED;
    end

    methods
        function obj = RayCasting3DMesh(varargin)
            %RAYCASTING3D Construct an instance of this class
            %   Initialize scanner, terrain and map parameters.

            % Call parent class constructor
            obj = obj@terrain.AbstractRayCasting3D(varargin{:});

        end

        function set.DTED(obj,db)
            % Scan data
            obj.DTED = db;
        end

        function showMap(obj)
            %SHOWMAP Plot the map and the terrain.
            [x, y, z] = deal(obj.DTED{:});

            obj.hFigure = findobj('Type','figure','Tag','MeshedTerrain');
            if isempty(obj.hFigure)
                obj.hFigure = figure;
                obj.hFigure.Tag = 'MeshedTerrain';
            end

            showMap@terrain.AbstractRayCasting3D(obj,x,y,z);

        end

        function [xs,ys,zs,xw,yw,zw] = raycast(obj, theta, psi_r)
            %RAYCAST Shooting a ray, ray casting, at an angle PSI from the
            %z-axis of ENU and THETA about the y-axis of the intermediate
            %frame.
            %
            %   The ray starts at the LiDAR sensor location and ends at the
            %   first hit with the terrain. It returns the x, y, z
            %   coordinates of the ray-triangle intersection point in the
            %   sensor frame. It also returns the same points in world
            %   frame for plotting purposes only.
            %
            %   Line Segment-to-Triangle Mesh Intersection: This is based
            %   on Tomas Möller & Ben Trumbore (1997) Fast, Minimum Storage
            %   Ray-Triangle Intersection, Journal of Graphics Tools, 2:1,
            %   21-28.    https://doi.org/10.1080/10867651.1997.10487468

            [xw,yw,zw] = deal(nan); [xs,ys,zs] = deal(nan);
            psi_s = obj.orientationLiDAR(1);
            phi_s = obj.orientationLiDAR(3);

            r0 = obj.positionLiDAR;
            D = terrain.AbstractRayCasting3D.wRsi(psi_s)*...
                terrain.AbstractRayCasting3D.siRs(phi_s)*... %? transpose
                terrain.AbstractRayCasting3D.sRi1(psi_r)*...
                terrain.AbstractRayCasting3D.i1Rr(theta)* [1;0;0];

            % Get a square patch of DTED determined by rayRange.
            ix = abs(obj.DTED{1} - r0(1)) <= obj.rayRange;
            iy = abs(obj.DTED{2} - r0(2)) <= obj.rayRange;
            X = obj.DTED{1}(ix);
            Y = obj.DTED{2}(iy);
            Z = obj.DTED{3}(iy,ix);

            % Get the single square mesh orthogonally below if THETA = 90;
            if theta == 90 && ~obj.flagScanAltimeter
                idx_x = find(X > r0(1),1);
                idx_y = find(Y > r0(2),1);
                if isempty(idx_x) || isempty(idx_y) || (idx_x == 1) || (idx_y == 1)
                    return;
                end
                X = X(idx_x-1:idx_x);
                Y = Y(idx_y-1:idx_y);
                Z = Z(idx_y-1:idx_y,idx_x-1:idx_x);
            end

            ni = numel(X);
            nj = numel(Y);
            if (nj == 0) || (ni == 0)
                return;
            end

            for j = 1:(nj-1)
                for i = 1:(ni-1)
                    i0  = i;  i1 = i+1; j0 = j;  j1 = j+1;
                    V0  = [X(i0); Y(j0); Z(j0,i0)];
                    V1  = [X(i1); Y(j0); Z(j0,i1)];
                    V2  = [X(i1); Y(j1); Z(j1,i1)];
                    V3  = [X(i0); Y(j1); Z(j1,i0)];

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
                                xw = V0(1) + u*E1(1) + v*E2(1);
                                yw = V0(2) + u*E1(2) + v*E2(2);
                                zw = V0(3) + u*E1(3) + v*E2(3);
                                % In sensor frame
                                xyz_s = transpose(terrain.AbstractRayCasting3D.siRs(phi_s,true)) * ...
                                        transpose(terrain.AbstractRayCasting3D.wRsi(psi_s,true)) * ...
                                        terrain.AbstractRayCasting3D.rTs(-obj.positionLiDAR) * [xw;yw;zw;1];
                                xs = xyz_s(1); ys =  xyz_s(2); zs =  xyz_s(3);
                                return;
                            end
                        end
                    end
                end
            end
        end
    end
end