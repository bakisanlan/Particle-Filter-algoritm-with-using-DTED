classdef DTED<handle
    properties
        mapArray
        mapInfo
        latList
        lonList
        LL0         = []
        flatMap     = []
    end
    methods
        function obj = DTED(varargin)
            if nargin == 0
                % Get DTED File Names
                d = dir(uigetdir("","Select folder that includes raw dted files."));
                dirNames = [];
                for i = 1:length(d)
                    curName = d(i).name;
                    if contains(curName ,'dt2') && ~contains(curName ,'meta')
                        dirNames = [dirNames;curName];
                    end
                end

                % Load DTED
                maxLat = -inf;
                minLat = +inf;
                maxLon = -inf;
                minLon = +inf;

                [rn,~] = size(dirNames);

                for i = 1:rn
                    curNames = dirNames(i,:);
                    [A{i},R{i}] = readgeoraster(curNames,"OutputType","double");

                    if minLat > min(R{i}.LatitudeLimits)
                        minLat = min(R{i}.LatitudeLimits);
                    end
                    if maxLat < max(R{i}.LatitudeLimits)
                        maxLat = max(R{i}.LatitudeLimits);
                    end

                    if minLon > min(R{i}.LongitudeLimits)
                        minLon = min(R{i}.LongitudeLimits);
                    end
                    if maxLon < max(R{i}.LongitudeLimits)
                        maxLon = max(R{i}.LongitudeLimits);
                    end
                    latList(i,:) = R{i}.LatitudeLimits;
                    lonList(i,:) = R{i}.LongitudeLimits;
                end

                % Arange Arrays
                uniqueLatList = unique(latList);
                uniqueLatList = uniqueLatList(end:-1:1);

                uniqueLonList = unique(lonList);

                smallMapCell = cell(length(uniqueLatList)-1,[length(uniqueLonList)-1]);

                for i = 1:length(R)
                    curLat = R{i}.LatitudeLimits(1,1);
                    curLon = R{i}.LongitudeLimits(1,1);
                    idx = find(curLat==uniqueLatList);
                    jdx = find(curLon==uniqueLonList);

                    smallMapCell{idx,jdx} = A{i};
                end
                finalA = cell2mat(smallMapCell);
                finalR = R{1};
                finalR.LatitudeLimits = [minLat,maxLat];
                finalR.LongitudeLimits = [minLon,maxLon];
                finalR.RasterSize = size(finalA);

                obj.mapInfo = finalR;
                obj.mapArray = finalA;
            else
                obj.mapArray = varargin{1};
                obj.mapInfo  = varargin{2};

            end
            obj.latList   = obj.mapInfo.LatitudeLimits(2) :obj.mapInfo.SampleSpacingInLatitude*(-1) : obj.mapInfo.LatitudeLimits(1);
            obj.lonList   = obj.mapInfo.LongitudeLimits(1):obj.mapInfo.SampleSpacingInLongitude: obj.mapInfo.LongitudeLimits(2);
 
        end
        function fig = Visualize(obj)
            latlim = obj.mapInfo.LatitudeLimits;
            lonlim = obj.mapInfo.LongitudeLimits;
            fig = figure;
            usamap(latlim,lonlim);
            geoshow(obj.mapArray,obj.mapInfo,"DisplayType","surface");
            demcmap(obj.mapArray);
        end
        function [croppedMapArray,croppedMapInfo] = Crop(obj,latMinMax,lonMinMax)
            latitudeList   = obj.latList;
            longitudeList  = obj.lonList;

            idxMax = find(latMinMax(1)<=latitudeList,1,"last");
            idxMin = find(latMinMax(2)>=latitudeList,1);
            jdxMin = find(lonMinMax(1)>=longitudeList,1,"last");
            jdxMax = find(lonMinMax(2)<=longitudeList,1);

            croppedMapInfo = obj.mapInfo;
            croppedMapArray = obj.mapArray(idxMin:idxMax,jdxMin:jdxMax);
            croppedMapInfo.LatitudeLimits = latMinMax;
            croppedMapInfo.LongitudeLimits = lonMinMax;
            croppedMapInfo.RasterSize = size(croppedMapArray);
        end
        function ConvertToFlat(obj,LL0)
            LLA = NaN(numel(obj.mapArray),3);% Creating place holder
            PSI0 = 0;
            HREF = 0;
            ct = 0;
            latitudeList    = obj.latList;
            longitudeList   = obj.lonList;
            for i = length(latitudeList):-1:1
                for j = 1:length(longitudeList)
                    ct = ct+1;
                    LLA(ct,1)= latitudeList(i);
                    LLA(ct,2)= longitudeList(j);
                    LLA(ct,3)= obj.mapArray(i,j);
                end
            end
            obj.LL0 = LL0;
            obj.flatMap = lla2flat( LLA,  LL0,  PSI0, HREF );
        end
        function SaveCSV(obj,fileName)
            writematrix(obj.flatMap, fileName);
        end
        function newCroppedDTEDList = SaveForUnity(obj,tileXNum,tileYNum)
            idx_range = floor(numel(obj.latList)/tileXNum);
            jdx_range = floor(numel(obj.lonList)/tileYNum);
            
            i_begin = 1;
            newCroppedDTEDList = cell(tileXNum,tileYNum);

            for i = 1:tileXNum
                i_end = i_begin + idx_range - 1;
                j_begin = 1;
                if i ~= tileXNum
                    latMinMax = [obj.latList(i_end),obj.latList(i_begin)];
                else
                    latMinMax = [obj.latList(end),obj.latList(i_begin)];
                end
                for j = 1:tileYNum
                    j_end = j_begin + jdx_range - 1;
                    if j ~= tileYNum
                        lonMinMax = [obj.lonList(j_begin),obj.lonList(j_end)];
                    else
                        lonMinMax = [obj.lonList(j_begin),obj.lonList(end)];
                    end

                    [croppedMap,croppedMapInfo] = obj.Crop(latMinMax,lonMinMax);
                    newCroppedDTED = DTED(croppedMap,croppedMapInfo);
                    % newCroppedDTED.Visualize();
                    newCroppedDTED.ConvertToFlat(obj.LL0);
                    
                    newCroppedDTEDList{i,j} = newCroppedDTED;

                    j_begin = j_end;

                    if (obj.LL0(1) > latMinMax(1) && obj.LL0(1) < latMinMax(2) && obj.LL0(2) > lonMinMax(1) && obj.LL0(2) < lonMinMax(2))
                        centered_i = i;
                        centered_j = j;
                    end
                end

                i_begin = i_end;
            end
            
            % Create Save Code 
            mkdir("Unity_DTED");
            for i = 1:tileXNum
                for j = 1:tileYNum
                    % locationCode = "U" + num2str(i-centered_i)+ "_R" + num2str(j-centered_j);
                    PosXstr = int2str(round(mean(newCroppedDTEDList{i,j}.flatMap(:,1),1)));
                    PosYstr = int2str(round(mean(newCroppedDTEDList{i,j}.flatMap(:,2),1)));
                    PosZstr = int2str(round(mean(newCroppedDTEDList{i,j}.flatMap(:,3),1)));
                    centerPositionCode = "CP_" + PosXstr + "_" + PosYstr + "_" + PosZstr;
                    sizeCode     = "SZ_" + replace(num2str(size(newCroppedDTEDList{i,j}.mapArray)),"  ","_");
                    heightCode   = "H_" + num2str(min(newCroppedDTEDList{i,j}.mapArray,[],"all")) + "_" + num2str(max(newCroppedDTEDList{i,j}.mapArray,[],"all"));
                    saveCode = "Unity_DTED\" + centerPositionCode + "_" + sizeCode + "_" + heightCode +".csv";
                    newCroppedDTEDList{i,j}.SaveCSV(saveCode);
                end
            end
     end
    end
end