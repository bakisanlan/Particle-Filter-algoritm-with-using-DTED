classdef DigitalElevationModel_simu < handle
    %DigitalElevationModel Manages a Digital Elevation Model (DEM) for
    %terrain scan and matching tasks.
    %   Processes a DTED file gathered by NASA SRTM.

    properties
        % Map dimensions
        mapWidth;

        % A and R
        A;          % Georeferenced image or data grid
        R;          % Spatial reference for the data grid A
        latitudes   % Latitudes for the data grid A
        longitudes  % Longitudes for the data grid 
    end

    methods
        function obj = DigitalElevationModel_simu(~,~)
            %DigitalElevationModel Constructs a DEM terrain object.
            
            % Load elevation data from file
            obj.mapWidth = 1000;
            lats = 40:41;
            lons = 27:29;
%             obj.loadData("dted_data/n41_e029_1arc_v3.dt2");
            obj.loadBatchData(lats, lons);
                        

        end

        function loadData(obj,filename)
            %loadData Load data from file
            [obj.A,obj.R] = readgeoraster(filename,"OutputType","double");
            obj.A = flip(obj.A);
            obj.latitudes   = transpose(...
                obj.R.LatitudeLimits(1):obj.R.SampleSpacingInLatitude:...
                obj.R.LatitudeLimits(2))';
            obj.longitudes = transpose(...
                obj.R.LongitudeLimits(1):obj.R.SampleSpacingInLongitude:...
                obj.R.LongitudeLimits(2));
        end

        function loadBatchData(obj, lats, lons)
            % load batch of latitudes longitudes
            A = [];
            obj.R = [];
            for lat=flip(lats)
                A_lat = [];
                for lon=lons
                    filename = "dted/n" + lat + "_e0" + lon +"_1arc_v3.dt2";
                    [A_i, R_i] = readgeoraster(filename,"OutputType","double");
                    A_lat = cat(2, A_lat, A_i);

                end
                A = cat(1, A, A_lat);
            end
            obj.A = flip(A);
            obj.R = R_i;

            obj.R.LatitudeLimits = [lats(1), lats(end)+1];
            obj.R.LongitudeLimits = [lons(1), lons(end)+1];
            obj.R.RasterSize = size(obj.A);
            obj.latitudes   = transpose(...
                obj.R.LatitudeLimits(1):obj.R.SampleSpacingInLatitude:...
                obj.R.LatitudeLimits(2))';
            obj.longitudes = transpose(...
                obj.R.LongitudeLimits(1):obj.R.SampleSpacingInLongitude:...
                obj.R.LongitudeLimits(2));
        end



        function [As,Rs,lats,lons] = slice(obj,lowerLeftPoint,upperRightPoint)
            %slice Return a slice of the loaded DTED data.
            latA = lowerLeftPoint(1);  lonA = lowerLeftPoint(2);
            latB = upperRightPoint(1); lonB = upperRightPoint(2);
            
            maxLong = lonB + obj.R.SampleSpacingInLongitude;
            maxLat  = latB + obj.R.SampleSpacingInLatitude;
            [~,maxLongIdx] = min(abs(obj.longitudes - maxLong));
            [~,maxLatIdx]  = min(abs(obj.latitudes - maxLat));

            minLong = lonA - obj.R.SampleSpacingInLongitude;
            minLat  = latA - obj.R.SampleSpacingInLatitude;
            [~,minLongIdx] = min(abs(obj.longitudes - minLong));
            [~,minLatIdx]  = min(abs(obj.latitudes - minLat));

            lons = obj.longitudes(minLongIdx:maxLongIdx);
            lats = obj.latitudes(minLatIdx:maxLatIdx);
            
            As = obj.A(minLatIdx:maxLatIdx,minLongIdx:maxLongIdx);
            Rs = obj.R;
            Rs.LatitudeLimits  = [min(lats) max(lats)];
            Rs.LongitudeLimits = [min(lons) max(lons)];
            Rs.RasterSize = size(As);
        end

        function [dted,x,y,z] = getMetricGridElevationMap(obj,...
                lowerLeftPoint,upperRightPoint, nDownSample)
            %createDTED Plot the map and the terrain.

            [As, ~, lats, lons] = slice(obj,lowerLeftPoint,upperRightPoint);
            z = As(1:nDownSample:end,1:nDownSample:end);
            lats = lats(1:nDownSample:end);
            lons = lons(1:nDownSample:end);
            [nLats, nLons] = size(z);

            % Note that while xf, and yf might be accurate, scaling
            % liinearly to the rest of the grid points is an approximation.
            xf = DigitalElevationModel.sphericalDistance([lats(1) lons(1)],[lats(1) lons(end)]);
            yf = DigitalElevationModel.sphericalDistance([lats(1) lons(1)],[lats(end) lons(1)]);
            x = (0:(nLons-1))'/(nLons-1)*xf;
            y = (0:(nLats-1))'/(nLats-1)*yf;
            dted = [{x}, {y}, {z}];
        end

        function [pNED] = getNEDCoordinates(obj,...
                lowerLeftPoint,upperRightPoint)
            %getNEDCoordinates gets North-East-Down coordinates of DTED
            
            
            [As, ~, lats, lons] = slice(obj,lowerLeftPoint,upperRightPoint);
            pNED.latitudes = reshape(lats, 1, []);
            pNED.longitudes = reshape(lons,1, []);
            pNED.altitudes = As;
            pNED.lat0 = 41.10068727249253;
            pNED.lon0 = 29.024573561608477;
            pNED.h0 = 0;
            pNED.ellipsoid = wgs84Ellipsoid();
            x_array = zeros(length(pNED.longitudes), length(pNED.latitudes));
            y_array = zeros(length(pNED.longitudes), length(pNED.latitudes));
            z_array = zeros(length(pNED.longitudes), length(pNED.latitudes));

            for i=1:length(lats)
                [x_array(:,i),y_array(:,i),z_array(:,i)] = geodetic2ned(pNED.latitudes(i), pNED.longitudes, pNED.altitudes(i,:), pNED.lat0, pNED.lon0, pNED.h0,  pNED.ellipsoid) ;
                
            end
            pNED.pN = x_array;
            pNED.pE = y_array;
            pNED.pD = z_array;
            
        end

        function visualizeDTED(obj)
            %visualizeDTED For testing purposes.
            %   This file loads and visualize DTED at (41N, 29E) of the
            %   Bosphorus.

            hF1 = figure; clf;
            usamap(obj.R.LatitudeLimits,obj.R.LongitudeLimits);
            geoshow(flip(obj.A),obj.R,"DisplayType","surface");
            cmap = demcmap(obj.A,16); colormap(hF1,cmap); colorbar;

            hF2 = figure; clf;
            [As, Rs, lats, lons] = slice(obj,[41 29],[41.30 29.20]);
            usamap(Rs.LatitudeLimits,Rs.LongitudeLimits);
            geoshow(flip(As),Rs,"DisplayType","surface");
            cmap = demcmap(As,16); colormap(hF2,cmap); colorbar;

            hF3 = figure; clf;
            mesh(lons, lats, As);
            pbaspect([1 1 0.1]); view(-7,31);
            axis([min(lons) max(lons) min(lats) max(lats)]);
            xlabel('Longitude');ylabel('Latitude');zlabel('Elevation');
            colormap(hF3,cmap); colorbar;
        end

    end

    methods(Static)
        function c = sphericalDistance(ptA,ptB)
            %sphericalDistance Distance between two points on the Earth's
            %surface specified in Geographic Coordinate System.
            %   Given Point A and Point B specified in terms of geodetic
            %   latitudes and longitudes (WGS 84), it returns in meters the
            %   shortest distance (great-circle distance). It implements
            %   Harvesine formula.

            EARTH_RADIUS = 6371*1000;  % earth radius

            % Point A
            latA = ptA(1)/180*pi;
            lonA = ptA(2)/180*pi;

            % Point B
            latB = ptB(1)/180*pi;
            lonB = ptB(2)/180*pi;

            % Haversine formula
            diff_lat = latB - latA;
            diff_lon = lonB - lonA;

            h = sin(diff_lat/2)^2+cos(latA)*cos(latB)*sin(diff_lon/2)^2;
            c = 2*EARTH_RADIUS*atan2(sqrt(h),sqrt(1-h)); % asin(sqrt(h))
        end
    end
end