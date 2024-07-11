classdef AbstractRayCasting3D < handle
    %AbstractRayCasting3D Abstract class providing default implmentations
    %for 3D ray casting.
    %
    %  Subclasses must provide implementations for the methods:
    %    raycast
    %
    %  Subclasses should overload showMap.
    %
    %  Subclasses should call:
    %
    %    obj = obj@terrain.AbstractRayCasting3D(varargin{:});
    %
    %   The world frame is assumed to be ENU. The sensor frame is a
    %   Forward-Left-Up (FLU) convention for x-y-z whose orientation is
    %   intialized to be aligned with ENU. Thus positive pitching is nose
    %   down.
    %
    %   The sensor frame's yaw angle is rotated psi_s degrees from the
    %   world frame (wROTsensor). Moreover, each LiDAR ray is in the
    %   direction of a unit vector that is rotated psi_r degrees about the
    %   z-axis of the sensor frame (sROT1) and then pitches theta degrees
    %   about the y-axis of the intermediate frame (1ROTr) leading to
    %   the ray frame.
    %
    %   POSE = [ROT  TRANSLATE   =  [1 0 0 tx *  [ROT  0
    %           0           1]       0 1 0 ty     0    1]
    %                                0 0 1 tz
    %                                0 0 0 1]
    %
    %   ROT       = wROTsensor * sROT1  * 1ROTr
    %   TRANSLATE = positionLiDAR
    %
    %   |r_w|        |r_r|
    %   |   | = POSE |   |
    %   |  1|        |  1|
    %
    %                   r_w = wROTsensor * sROT1 * 1ROTr * r_r
    %   (wROTsensor)^-1 r_w = sROT1      * 1ROTr * r_r
    %
    %   wROTsensor = [cos(psi_s)  -sin(psi_s)  0
    %                 sin(psi_s)   cos(psi_s)  0
    %                 0          0         1]
    %
    %   sROT1      = [cos(psi_r)  -sin(psi_r)  0
    %                 sin(psi_r)   cos(psi_r)  0
    %                 0          0             1]
    %
    %   1ROTr      = [cos(theta)  0  sin(theta)
    %                 0           1  0
    %                -sin(theta)  0  cos(theta)]
    %
    %   POSE = [cos(psi_s+psi_r)cos(theta) -sin(psi_s+psi_r)  cos(psi_s+psi_r)sin(theta) tx
    %           sin(psi_s+psi_r)cos(theta)  cos(psi_s+psi_r)  sin(psi_s+psi_r)sin(theta) ty
    %          -sin(theta)                  0                 cos(theta)                 tz
    %           0                           0                 0                          1 ]

    properties
        positionLiDAR;    % x, y, z         (ENU world frame)
        orientationLiDAR; % psi-theta-phi   (Yaw angle of sensor frame w.r.t. to world frame)
        rayRange;
        dpsi;
        dtheta;
        ptCloud;
        hFigure;
        TFRptCloud
        flagScanAltimeter

    end

    methods
        function obj = AbstractRayCasting3D(varargin)
            %AbstractRayCasting3D Construct an instance of this class.
            %   Initializes common parameters. It takes up to 5 parameters
            %   in the following order (positionLiDAR, orientationLiDAR,
            %   rayRange, dpsi, dtheta).

            if nargin < 1
                % Default argument
                % Dimensions and scanner parameters
                obj.positionLiDAR = [0;0;50];
                obj.orientationLiDAR = [45; 0; 0];
                obj.rayRange = 1000;
                obj.dpsi = 1;
                obj.dtheta = 1;
                obj.flagScanAltimeter = false; % default use point cloud RADAR
            else
                obj.flagScanAltimeter = false;
                obj.positionLiDAR = varargin{1};
                if nargin >= 2
                    obj.orientationLiDAR = varargin{2};
                    if nargin >= 3
                        obj.rayRange = varargin{3};
                        if nargin >= 4
                            obj.dpsi = varargin{4};
                            if nargin >= 5
                                obj.dtheta = varargin{5};
                            end
                        end
                    end
                end
            end
        end

        function [Xs,Ys,Zs,Xw,Yw,Zw] = sweep_line(obj, theta_0, theta_f, psi_r)
            %SWEEP_LINE Scans a line
            %   Scans a line starting at theta_0 and ending at theta_f.
            theta = theta_0:obj.dtheta:theta_f;
            n = numel(theta);
            ZERO = zeros(n,1);
            Xs = ZERO; Ys = ZERO; Zs  = ZERO;
            Xw = ZERO; Yw = ZERO; Zw = ZERO;
            for i = 1:n
                [xr,yr,zr,xw,yw,zw] = raycast(obj, theta(i),psi_r);
                Xs(i) = xr; Ys(i) = yr;  Zs(i) = zr;
                Xw(i) = xw; Yw(i) = yw;  Zw(i) = zw;
            end
        end

        function [Xs,Ys,Zs,Xw,Yw,Zw] = sweep_arc(obj, psi_r, theta)
            %SWEEP_ARC Scans a symmetric arc.
            %   Scans a symmetric arc around current psi provided by
            %   dpsi.
            PSI_r = (-psi_r):obj.dpsi:psi_r;
            m = numel(PSI_r);
            ZERO = zeros(m,1);
            Xs = ZERO; Ys = ZERO; Zs = ZERO;
            Xw = ZERO; Yw = ZERO; Zw = ZERO;
            for j = 1:m
                [xr,yr,zr,xw,yw,zw] = raycast(obj, theta, PSI_r(j));
                Xs(j) = xr; Ys(j) = yr;  Zs(j) = zr;
                Xw(j) = xw; Yw(j) = yw;  Zw(j) = zw;
            end
        end

        function scanTerrain(obj, flagPlot)
            %SCANREALTERRAIN Scan the terrain.
            %
            %   Two types of scanning are possible. Scanning a terrain that
            %   functions as the pre-loaded reference map terrain whose
            %   scanning can be compared with a radar scan, or as a terrain
            %   that functions as the simulated enviroment real terrain
            %   emulating a real radar scan.
            %
            %   When used as a radar emulator scanning the simulated
            %   enviroment terrain. It is best to apply ray casting that
            %   accounts for uncertainities encountered by a real radar
            %   scan.

            %Set flagAltimeter false when scanning terrain with point cloud
            % RADAR rather than single point ALTIMETER
            obj.flagScanAltimeter = false;

            if nargin == 1
                flagPlot = false;
            end

            [x1,y1,z1,xw1,yw1,zw1] = obj.sweep_line(30,90, 45);
            [x2,y2,z2,xw2,yw2,zw2] = obj.sweep_line(30,90, 0);
            [x3,y3,z3,xw3,yw3,zw3] = obj.sweep_line(30,90, -45);
            [x4,y4,z4,xw4,yw4,zw4] = obj.sweep_arc(170, 60);
            [x5,y5,z5,xw5,yw5,zw5] = obj.sweep_arc(170, 40);
            [x6,y6,z6,xw6,yw6,zw6] = obj.sweep_arc(170, 30);

            Xs = [x1;x2;x3;x4;x5;x6];
            Ys = [y1;y2;y3;y4;y5;y6];
            Zs = [z1;z2;z3;z4;z5;z6];

            Xw = [xw1;xw2;xw3;xw4;xw5;xw6];
            Yw = [yw1;yw2;yw3;yw4;yw5;yw6];
            Zw = [zw1;zw2;zw3;zw4;zw5;zw6];

            obj.ptCloud.s = pointCloud([Xs Ys Zs]);
            obj.ptCloud.w = pointCloud([Xw Yw Zw]);

            if flagPlot && ~isempty(obj.hFigure)
                figure(obj.hFigure);
                plot3(Xw,Yw,Zw,'r.','MarkerSize',10,'DisplayName','LiDAR Scans');
            end
        end

        function scanAltimeter(obj)
            %scanAltimeter scan the terrain just using radar altimeter
            %signal

            %Set flagAltimeter true when just using altimeter
            obj.flagScanAltimeter = true;
            [xs,ys,zs,xw,yw,zw]      = raycast(obj, 90, 0);

            obj.ptCloud.s = pointCloud([xs ys zs]);
            obj.ptCloud.w = pointCloud([xw yw zw]);

        end

        function [zs, zw] = readAltimeter(obj, location_s)
            %READALTIMETER Returns altitude above terrain from a given
            %location in sensor frame.
            %
            %   Given an x-y-z location in sensor frame, it returns the
            %   orthogonal distance to terrain in sensor frame, namely:
            %
            %      zs = - altitude
            %

            psi_s = obj.orientationLiDAR(1);
            phi_s = obj.orientationLiDAR(3);

            % In world frame
            xyz_w = terrain.AbstractRayCasting3D.rTs(obj.positionLiDAR) * ...
                terrain.AbstractRayCasting3D.wRsi(psi_s,true) * ...
                terrain.AbstractRayCasting3D.siRs(phi_s,true) * [location_s;1];

            % Move to radar point on z direction of world ENU to aircraft
            % level
            xyz_w(3) = obj.positionLiDAR(3);

            % Save the sensor location and range
            originalPos     = obj.positionLiDAR;
            originalOrient  = obj.orientationLiDAR;
            originalRange   = obj.rayRange;

            % Move the sensor to the desired point and ray cast from there
            obj.positionLiDAR   = xyz_w(1:3);
            % Force the sensor to no bank angle
            obj.orientationLiDAR(3) = 0;
            %obj.rayRange        = obj.positionLiDAR(3);  % limits terrain area to a minimum
            [xs,ys,zs,xw,yw,zw]      = raycast(obj, 90, 0);
            obj.ptCloud.s = pointCloud([xs ys zs]);
            obj.ptCloud.w = pointCloud([xw yw zw]);

            % Restore the sensor location and range
            obj.positionLiDAR   = originalPos;
            obj.orientationLiDAR = originalOrient;
            obj.rayRange        = originalRange;
        end

        function [zs, zw] = readAltimeter_interp(obj, location_s)
            % readAltimeter_interp Returns altitude above terrain from a given
            % location in sensor frame with using built in interp2 function
            %
            %   Given an x-y-z location in sensor frame, it returns the
            %   orthogonal distance to terrain in sensor frame, namely:
            %
            %      zs = - altitude
            %

            psi_s = obj.orientationLiDAR(1);
            phi_s = obj.orientationLiDAR(3);

            % Convert the rows of the array into cells
            % location_s is Nx3 position matrix for radar point in sensor
            % frame

            [row, col] = size(location_s);
            location_s_cell = mat2cell(location_s, ones(1, row), col);
            % Apply the wPOSEs function to each row (cell)
            resultCellArray = cellfun(@(x) terrain.AbstractRayCasting3D.wPOSEs(obj.positionLiDAR,x,psi_s,phi_s), location_s_cell, 'UniformOutput', false);
            % Convert the cell array back to a matrix
            resultArray = cell2mat(resultCellArray);
            resultArray = reshape(resultArray,col+1,row);
            location_w = resultArray';
            location_w = location_w(:,1:3);

            [xs,ys,zs,xw,yw,zw] = sliding(obj,location_w);
            obj.ptCloud.s = pointCloud([xs ys zs]);
            obj.ptCloud.w = pointCloud([xw yw zw]);
        end

        function scanTFRmode(obj, flagPlot)
            % scanTFRmode scan the terrain with just sweep line that is
            % front view of aircraft bounded [-8,32] theta degrees.

            if nargin == 1
                flagPlot = false;
            end

            [x1,y1,z1,xw1,yw1,zw1] = obj.sweep_line(-8,32, 0);
            % [x2,y2,z2,xw2,yw2,zw2] = obj.sweep_line(30,90, 0);
            % [x3,y3,z3,xw3,yw3,zw3] = obj.sweep_line(30,90, -45);
            % [x4,y4,z4,xw4,yw4,zw4] = obj.sweep_arc(170, 60);
            % [x5,y5,z5,xw5,yw5,zw5] = obj.sweep_arc(170, 40);
            % [x6,y6,z6,xw6,yw6,zw6] = obj.sweep_arc(170, 30);

            Xs = x1;
            Ys = y1;
            Zs = z1;

            Xw = xw1;
            Yw = yw1;
            Zw = zw1;

            obj.TFRptCloud.s = pointCloud([Xs Ys Zs]);
            obj.TFRptCloud.w = pointCloud([Xw Yw Zw]);


            if flagPlot && ~isempty(obj.hFigure)
                figure(obj.hFigure);
                plot3(Xw,Yw,Zw,'r.','MarkerSize',10,'DisplayName','LiDAR Scans');
            end
        end

        function showMap(obj,x,y,z)
            %SHOWMAP Plot the map and the terrain.

            figure(obj.hFigure);clf(obj.hFigure); hold on; legend;
            xlabel('x');ylabel('y');
            view(-25,10); pbaspect([1 1 1]); daspect([1 1 1]);

            surf(x,y,z,'DisplayName','Terrain','EdgeColor', [0.4660 0.6740 0.1880],'FaceColor','green','FaceLighting','gouraud','FaceAlpha',0.90);
            title('Terrain Scanning by LiDAR');

            axis([min(x,[],"all") max(x,[],"all") min(y,[],"all") max(y,[],"all") min(z,[],"all") obj.positionLiDAR(3)]);

            % Plot the LiDAR location
            plot3(obj.positionLiDAR(1),obj.positionLiDAR(2),obj.positionLiDAR(3),'k.','MarkerSize',20,'DisplayName','LiDAR Location')
        end

    end

    methods (Abstract = true)

        % Ray cast implementation
        [xs,ys,zs,xw,yw,zw] = raycast(obj, theta, psi_r);

    end

    methods(Static, Sealed)
        function r = wRsi(psi_s, augmentedflag)
            % Rotation matrix from intermediate sensor frame to world frame. It is given
            % as follows:
            %
            %   wRsi = [cos(psi_s)  -sin(psi_s)  0
            %          sin(psi_s)   cos(psi_s)  0
            %          0            0           1]
            %
            %  The matrix can be augmented to accomodate for translations.

            psi_s = psi_s/180*pi;

            r=[ cos(psi_s) -sin(psi_s) 0;
                sin(psi_s)  cos(psi_s) 0;
                0           0          1];
            if (nargin == 2) && augmentedflag
                r=  [r zeros(3,1); zeros(1,3) 1];
            end
        end

        function r = siRs(phi_s, augmentedflag)
            % Rotation matrix from sensor frame to intermediate sensor frame. It is given
            % as follows:
            %
            %   siRs = [1   0           0
            %           0   cos(phi_s)  sin(phi_s)
            %           0  -sin(phi_s)  cos(phi_s)]
            %
            %  The matrix can be augmented to accomodate for translations.

            phi_s = phi_s/180*pi;

            r = [1   0            0
                0   cos(phi_s)  -sin(phi_s)
                0   sin(phi_s)   cos(phi_s)];

            if (nargin == 2) && augmentedflag
                r=  [r zeros(3,1); zeros(1,3) 1];
            end
        end

        function r = sRi1(psi_r, augmentedflag)
            % Rotation matrix from intermediate frame 1 to sensor frame,
            % and is given by:
            %
            %   sRi1 = [cos(psi_r)  -sin(psi_r)  0
            %           sin(psi_r)   cos(psi_r)  0
            %           0          0             1]
            %
            %  The matrix can be augmented to accomodate for translations.

            psi_r = psi_r/180*pi;

            r=[ cos(psi_r) -sin(psi_r) 0;
                sin(psi_r)  cos(psi_r) 0;
                0           0          1];

            if (nargin == 2) && augmentedflag
                r=  [r zeros(3,1); zeros(1,3) 1];
            end
        end

        function r = i1Rr(theta, augmentedflag)
            % Rotation matrix from ray frame to intermediate frame 1, and
            % is given as follows:
            %
            %   i1Rr      = [cos(theta)  0  sin(theta)
            %                0           1  0
            %               -sin(theta)  0  cos(theta)]
            %
            %  The matrix can be augmented to accomodate for translations.

            theta = theta/180*pi;

            r=[ cos(theta)   0   sin(theta);
                0            1   0;
                -sin(theta)   0   cos(theta)];

            if (nargin == 2) && augmentedflag
                r=  [r zeros(3,1); zeros(1,3) 1];
            end
        end

        function t = rTs(d)
            % Displacement from sensor frame to ray frame, and is given as
            % follows:
            %
            %  rTs = [1 0 0 tx
            %         0 1 0 ty
            %         0 0 1 tz
            %         0 0 0 1]

            t= [eye(3) d; zeros(1,3) 1];
        end

        function pose = wPOSEr(d,psi_s,psi_r,theta)
            % Pose from world frame to ray frame, and is given by:
            %
            %   POSE = rTs * [wRs 0;0 1] * [sRi1 0;0 1] * [i1Rr 0;0 1];
            %
            %   POSE = [cos(psi_s+psi_r)cos(theta) -sin(psi_s+psi_r)  cos(psi_s+psi_r)sin(theta) tx
            %           sin(psi_s+psi_r)cos(theta)  cos(psi_s+psi_r)  sin(psi_s+psi_r)sin(theta) ty
            %          -sin(theta)                  0                 cos(theta)                 tz
            %           0                           0                 0                          1 ]
            %
            %
            wRs         = terrain.AbstractRayCasting3D.wRs(psi_s, true);
            sRi1        = terrain.AbstractRayCasting3D.sRi1(psi_r, true);
            i1Rr        = terrain.AbstractRayCasting3D.i1Rr(theta, true);
            rTs         = terrain.AbstractRayCasting3D.rTs(d);

            pose =  rTs * wRs * sRi1 * i1Rr;
        end

        function pose_w = wPOSEs(positionLiDAR,pose_s,psi_s,phi_s)
            % Pose sensor frame to world frame , and is given by:

            if length(pose_s(1,:)) ~= 1
                pose_s = pose_s';
            end
            pose_w = terrain.AbstractRayCasting3D.rTs(positionLiDAR) * ...
                terrain.AbstractRayCasting3D.wRsi(psi_s,true) * ...
                terrain.AbstractRayCasting3D.siRs(phi_s,true) * [pose_s;1];
        end

        function pose_s = sPOSEw(positionLiDAR,pose_w,psi_s,phi_s)
            % Pose sensor frame to world frame , and is given by:

            if length(pose_w(1,:)) ~= 1
                pose_w = pose_w';
            end
            pose_s = terrain.AbstractRayCasting3D.siRs(phi_s,true) * ...
                terrain.AbstractRayCasting3D.wRsi(psi_s,true) * ...
                terrain.AbstractRayCasting3D.rTs(-positionLiDAR) * [pose_w;1];
        end
    end
end