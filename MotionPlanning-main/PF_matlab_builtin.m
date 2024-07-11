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
psi0    = 0*pi/180;
dt      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
phi_r = 0;
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = dt;
hAircraft.WithNoise     = true;      % Enables wind disturbance
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

initialize(pf, 100, state_bounds);
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @exampleHelperCarBotStateTransition;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @exampleHelperCarBotMeasurementLikelihood;

% Last best estimation for x, y and theta
lastBestGuess = hAircraft.Pose';

%% Game loop

simulationTime = 0;
tf = 200;

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

    lastBestGuess = stateCorrected(1:3);

    % % Update plot
    % if ~isempty(get(groot,'CurrentFigure')) % if figure is not prematurely killed
    %     updatePlot(carbot, pf, lastBestGuess, simulationTime);
    % else
    %     break
    % end

    waitfor(r);
    
    % Update simulation time
    simulationTime = simulationTime + dt;
end