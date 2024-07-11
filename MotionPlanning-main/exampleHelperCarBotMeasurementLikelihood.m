function  likelihood = exampleHelperCarBotMeasurementLikelihood(pf, particles, ptCloudRadar, varargin) %#ok<INUSL>

alt_std = 3;
hReferenceMapScanner= varargin{1};
priorPose = varargin{2};
flagRAYCAST = varargin{3};

N = length(particles(:,1));

hReferenceMapScanner.positionLiDAR    = priorPose(1:3,1);
hReferenceMapScanner.orientationLiDAR = priorPose(4:6,1);

MAD = zeros(N,1);

for i=1:N

    % true altitude point cloud data
    Zr = ptCloudRadar.s.Location(:,3);
    Zr_w = ptCloudRadar.w.Location(:,3);
    n = numel(Zr);

    x_particle = particles(i,1);
    y_particle = particles(i,2);
    hReferenceMapScanner.positionLiDAR([1, 2]) = [x_particle y_particle];
    %disp(self.hReferenceMapScanner.positionLiDAR)
    if flagRAYCAST

        % When Radar use RadarAltimeter for scaning terrain,
        % ReferenceMapScanner also should use RadarAltimeter
        if length(Zr) ~= 1
            hReferenceMapScanner.scanTerrain(false);
        else
            hReferenceMapScanner.scanAltimeter;
        end
        Zs = hReferenceMapScanner.ptCloud.s.Location(:,3);
        Zw = hReferenceMapScanner.ptCloud.w.Location(:,3);

        particle_pc_w = hReferenceMapScanner.ptCloud.w.Location(:,1:2);
    else
        XrYr_s = ptCloudRadar.s.Location(:,1:2);
        XrYr_w = zeros(n,2);

        Zs = zeros(n,1);
        Zw = zeros(n,1);

        for j=1:n
            [Zs(j), Zw(j)] = hReferenceMapScanner.readAltimeter([XrYr_s(j,1);XrYr_s(j,2);Zr(j)]);
            XrYr_w(j,:) = hReferenceMapScanner.ptCloud.w.Location(:,1:2);
        end
        %
        % [Zs, Zw] = self.hReferenceMapScanner.readAltimeter_interp([XrYr_s self.Zr]);
        % XrYr_w = self.hReferenceMapScanner.ptCloud.w.Location(:,1:2);
        % %Z(j) = self.hReferenceMapScanner.readAltimeter([XrYr(j,1);XrYr(j,2);0]);

        particle_pc_w = XrYr_w;
    end

    % TO RESOLVE LATER. Decide the best way to treat NANs.
    idx = or(isnan(Zw),isnan(Zr_w));
    flag = 1;
    if flag == 0
        % Using world frame, with NaNs replaced by zeros
        Z = Z + hReferenceMapScanner.positionLiDAR(3); Zr = Zr + hReferenceMapScanner.positionLiDAR(3);
        Z(idx) = 0;
        Zr(idx) = Z(idx);
    elseif flag == 1
        % Deleting NaNs
        Zw(idx) = [];
        Zr_w(idx) = [];
        %size(particle_pc)
        particle_pc_w(idx,:) = [];
        %size(particle_pc)
    elseif flag == 2
        % Using local frame with NaNs replaced by zero w.r.t. sensor frame
        Z(idx) = 0 - self.hReferenceMapScanner.positionLiDAR(3);
        self.Zr(idx) = Z(idx);
    end

    if ~isempty(Zr_w)
        MAD(i,1) = mean(abs(Zw - Zr_w),1);
    else
        MAD(i,1) = 999;
    end

likelihood = (1 / (alt_std*sqrt(2*pi))) .* exp(-0.5 .* (MAD./ alt_std).^2);
end
