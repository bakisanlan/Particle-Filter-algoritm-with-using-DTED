% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clc; clear; close all;
rng('default');


%% Create simulation objects
hDEM                    = terrain.DigitalElevationModel;
hRadar                  = terrain.RayCasting3DMesh;
hReferenceMapScanner    = terrain.RayCasting3DMesh;
hAircraft               = terrain.AircraftBot(zeros(4,1),0);

%% Settings
% Radar settings
hRadar.dtheta   = 10;
hRadar.dpsi     = 10;
hRadar.rayRange = 1500;

% DEM settings
% Downsampled by 10, thus 300m resolution
hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
hRadar.showMap; axis('auto');
hRadar.hFigure.CurrentAxes.Children(2).FaceColor = 'flat'; view(2);
colorbar;
set(hRadar.hFigure,'WindowStyle','docked')
hLocationPoint = hRadar.hFigure.Children(3).Children(1); % handle to aircraft location
hLocationPoint.Marker = "^";
hLocationPoint.MarkerSize = 10;
hLocationPoint.MarkerFaceColor = "cyan";

% Reference map scanner settings
hReferenceMapScanner.dtheta     = hRadar.dtheta;
hReferenceMapScanner.dpsi       = hRadar.dpsi;
hReferenceMapScanner.rayRange   = hRadar.rayRange;
hReferenceMapScanner.DTED       = hRadar.DTED;

% Aircraft settings
% Initial pose
x0      = 1500;
y0      = 1500;
z0      = 500;
psi0    = 20*pi/180;
Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;

TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];

% Estimator settings
hEstimator = terrain.StateEstimatorPF(1000,hAircraft.Pose,500,500,0,3,Ts);
hEstimator.hReferenceMapScanner = hReferenceMapScanner;


%% Game Loop
%r = rateControl(1/Ts);
%reset(r);
simTime = 0;
while simTime < 200

    u = [100; 2*pi/500];
    hAircraft.move(u);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain(false);

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    param = hEstimator.getEstimate(u,hRadar.ptCloud, true);
    toc

    hAircraft.EstimatedPose = [param(1); param(2)];
    disp (['Estimated [x,y]: ' '[ ' num2str(hAircraft.EstimatedPose') ' ]']);


    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow

    %waitfor(r);

    TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
    TraceEstimatedPose = [TraceEstimatedPose, hAircraft.EstimatedPose];
    
    % Update simulation time
    simTime = simTime + Ts;
end

figure(2); hold on;
plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',5);
legend('Truth', 'Estimated');
