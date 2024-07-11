% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clc; clear; close all;
addpath(genpath('C:\Users\user\Desktop\githubdeneme\pointssim'))
rng(5,'twister')
%% Create simulation objects
scene = 'BP';
hDEM                    = terrain.DigitalElevationModel(scene);
hRadar                  = terrain.RayCasting3DMesh;
hReferenceMapScanner    = terrain.RayCasting3DMesh;
hAircraft               = terrain.AircraftBot(zeros(4,1),0);

%% Settings
% Radar settings
% hRadar.dtheta   = -2.5;
% hRadar.dpsi     = 0;
% hRadar.rayRange = 1500;
hRadar.dtheta   = 10;
hRadar.dpsi     = 10;
hRadar.rayRange = 1500;

% DEM settings
% Downsampled by 10, thus 300m resolution
% bosphorus
if scene == 'BP'
    left_lower = [41 29];
    right_upper = [41.30 29.20];
    hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
    modelF = 1;
elseif scene == 'GC'
    left_lower = [36.18777778 -112.54111111];
    right_upper = [36.38000000 -112.31166667];
    hRadar.DTED = hDEM.getMetricGridElevationMap(left_lower,right_upper, 10);
    hRadar.positionLiDAR(3) = 3000;
    modelF = 2;
end

hRadar.showMap; axis('auto');
hRadar.hFigure.CurrentAxes.Children(2).FaceColor = 'flat'; view(2);
colorbar;
%set(hRadar.hFigure,'WindowStyle','docked')
hLocationPoint = hRadar.hFigure.Children(3).Children(1); % handle to aircraft location
hLocationPoint.Marker = "^";
hLocationPoint.MarkerSize = 10;
hLocationPoint.MarkerFaceColor = "cyan";
%ginput()
%%
% Reference map scanner settings
hReferenceMapScanner.dtheta     = hRadar.dtheta;
hReferenceMapScanner.dpsi       = hRadar.dpsi;
hReferenceMapScanner.rayRange   = hRadar.rayRange;
hReferenceMapScanner.DTED       = hRadar.DTED;

x0      = 1500;
y0      = 1500;
z0      = 300;
psi0    = 20*pi/180;
dt      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
phi_r = 0;
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = dt;
hAircraft.WithNoise     = false;      % Enables wind disturbance
flagRAYCAST = false;

TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];
var = [];

%% PF Inıt

pf = stateEstimatorPF;
span_partc = 500;
x_range_partc = [hAircraft.Pose(1)-0.5*span_partc(1) hAircraft.Pose(1)+0.5*span_partc(1)];
y_range_partc = [hAircraft.Pose(2)-0.5*span_partc(1) hAircraft.Pose(2)+0.5*span_partc(1)];
z_range_partc = [hAircraft.Pose(3) hAircraft.Pose(3)];
psi_range_partc = [hAircraft.Pose(4) hAircraft.Pose(4)];
state_bounds = [x_range_partc ; y_range_partc ; z_range_partc ; psi_range_partc];

N = 100;
initialize(pf, N, state_bounds);
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @particle_step;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @particles_likelihood;

% Last best estimation for x, y and theta
lastBestGuess = hAircraft.Pose';

%% Game loop

simulationTime = 0;
tf = 200;
i=1;
while simulationTime < tf % if time is not up

    % Generate motion command that is to be sent to the robot
    % NOTE there will be some discrepancy between the commanded motion and the
    % motion actually executed by the robot.
    uCmd = [100; 2*pi/500];

    hAircraft.move(uCmd,1);

    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain;

    measurement = hRadar.ptCloud;

    tic;
    % Predict the carbot pose based on the motion model
    [statePred, covPred] = predict(pf, dt, uCmd);


    % If measurement is available, then call correct, otherwise just use
    % predicted result
    if ~isempty(measurement)
        [stateCorrected, covCorrected] = correct(pf, measurement,hReferenceMapScanner,[hRadar.positionLiDAR; hRadar.orientationLiDAR],flagRAYCAST);
    else
        stateCorrected = statePred;
        covCorrected = covPred;
    end

    toc;

    lastBestGuess = stateCorrected(1:2);
    disp (['Estimated [x,y]: ' '[ ' num2str(lastBestGuess(1:2)) ' ]' newline]);


    particles_history(1+N*(i):N*(i+1),:) = pf.Particles(:,1:2);
    hAircraft.EstimatedPose = [lastBestGuess(1); lastBestGuess(2)];
    TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
    TraceEstimatedPose = [TraceEstimatedPose, hAircraft.EstimatedPose];

    % Update simulation time
    simulationTime = simulationTime + dt;
    i = i+1;
end
%%
valid_index = ~isnan(TraceEstimatedPose);
TracePose = TracePose(1:2,:);
diff = reshape(TracePose(logical([[0; 0] valid_index])),2,[]) - reshape(TraceEstimatedPose(valid_index),2,[]);

errors = sqrt(diff(1,:).^2 + diff(2,:).^2);
mean_error = mean(errors);
%mean_error_slid = mean(sqrt(diff_slid(1,:).^2 + diff_slid(2,:).^2));

%mean_std = mean(sqrt(var));
mean_std = sqrt(sum((errors - mean_error).^2)/N);
%mean_std_slid = mean(sqrt(var_slid));

disp(['Estimation error of PF ',num2str(mean_error),' meters mean and ',num2str(mean_std),' std'])

figure(2); hold on;
plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',5);
plot(particles_history(:,1),particles_history(:,2),'k.')
daspect([1 1 1])
pbaspect([1 1 1])
legend('Truth', 'Estimated');


%% Motion Prediction for Particles

function predictParticles = particle_step(pf, prevParticles, dt, u) %#ok<INUSL>
%exampleHelperCarBotStateTransition Propagate particles based on given motion model.

%   Copyright 2015-2016 The MathWorks, Inc.

process_std = [5 0.02];
N = length(prevParticles(:,1));

u = reshape(u, numel(u),1); % column vector
noise = (randn(N, 2) .* process_std);
u = u' + noise; %

% Updated based on input

% different model structure
dPose1_dt = u(:,1) .* cos(prevParticles(:,4));
dPose2_dt = u(:,1) .* sin(prevParticles(:,4));
dPose3_dt = 0;
dPose4_dt = u(:,2);

predictParticles(:, 1) = prevParticles(:, 1) + dt .* dPose1_dt;
predictParticles(:, 2) = prevParticles(:, 2) + dt .* dPose2_dt;
predictParticles(:, 3) = prevParticles(:, 3) + dt .* dPose3_dt;
predictParticles(:, 4) = prevParticles(:, 4) + dt .* dPose4_dt;

end

%% Measurement Function for Particles
function  likelihood = particles_likelihood(pf, particles, ptCloudRadar, varargin) %#ok<INUSL>

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
end
