% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clear; close all;
rng('default');


%% Create simulation objects
hDEM                    = terrain.DigitalElevationModel;
hRadar                  = terrain.RayCasting3DMesh;
hReferenceMapScanner    = terrain.RayCasting3DMesh;
hEstimator              = terrain.StateEstimator;
hAircraft               = terrain.AircraftBot(zeros(4,1),0);

%% Settings
% Radar settings
hRadar.dtheta   = 10;
hRadar.dpsi     = 10;
hRadar.rayRange = 4000;

% DEM settings
% Downsampled by 10, thus 300m resolution
%hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
left_lower = [36.18777778 -112.54111111];
right_upper = [36.38000000 -112.31166667];
%hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
hRadar.DTED = hDEM.getMetricGridElevationMap(left_lower,right_upper, 10);

% hRadar.showMap; axis('auto');
% hRadar.hFigure.CurrentAxes.Children(2).FaceColor = 'flat'; view(2);
% colorbar;
% set(hRadar.hFigure,'WindowStyle','docked')
% hLocationPoint = hRadar.hFigure.Children(3).Children(1); % handle to aircraft location
% hLocationPoint.Marker = "^";
% hLocationPoint.MarkerSize = 10;
% hLocationPoint.MarkerFaceColor = "cyan";



% Reference map scanner settings
hReferenceMapScanner.dtheta     = hRadar.dtheta;
hReferenceMapScanner.dpsi       = hRadar.dpsi;
hReferenceMapScanner.rayRange   = hRadar.rayRange;
hReferenceMapScanner.DTED       = hRadar.DTED;

% Estimator settings
hEstimator.hReferenceMapScanner = hReferenceMapScanner;

% Aircraft settings
% Initial pose
x0      = 1500;
y0      = 1500;
z0      = 2500;
psi0    = 20*pi/180;
Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;

TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];

%% Game Loop
r = rateControl(1/Ts);
reset(r);
simTime = 0;
while simTime < 200

    u = [100; 2*pi/500];
    hAircraft.move(u);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain(false);

    % Estimate using x-y grid overlaying and not full ray casting
    param = hEstimator.getEstimate( ...
        [hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
        hRadar.ptCloud, true);

    hAircraft.EstimatedPose = [param(1); param(2)];

    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow

    waitfor(r);

    TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
    TraceEstimatedPose = [TraceEstimatedPose, hAircraft.EstimatedPose];
    
    % Update simulation time
    simTime = simTime + Ts;
end

figure(2); hold on;
plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',5);
legend('Truth', 'Estimated');
