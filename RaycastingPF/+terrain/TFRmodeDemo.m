% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

% https://www.youtube.com/watch?v=zqJvxHpkbvM&t=460s
clc; clear; close all;
rng(5,'twister')
%% Create simulation objects
scene = 1;
hDEM                    = terrain.DigitalElevationModel(scene);
hTFRadar                = terrain.RayCasting3DMesh;
hReferenceMapScanner    = terrain.RayCasting3DMesh;
hAircraft               = terrain.AircraftBot(zeros(4,1),0);

%% Settings
% Radar settings
hTFRadar.dtheta   = 1;
hTFRadar.dpsi     = 1;
hTFRadar.rayRange = 11000;

left_lower = [41 29];  % 33000, 17000
right_upper = [41.30 29.20];
hTFRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
modelF = 1;

% hTFRadar.showMap; axis('auto');
% hTFRadar.hFigure.CurrentAxes.Children(2).FaceColor = 'flat'; view(2);
% colorbar;
% %set(hRadar.hFigure,'WindowStyle','docked')
% hLocationPoint = hTFRadar.hFigure.Children(3).Children(1); % handle to aircraft location
% hLocationPoint.Marker = "^";
% hLocationPoint.MarkerSize = 10;
% hLocationPoint.MarkerFaceColor = "cyan";

%% Aircraft Settings
x0      = 1500;
y0      = 1500;
z0      = 300;
psi0    = 60*pi/180;

Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
hTFRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hTFRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;
hAircraft.WithNoise     = false;      % Enables wind disturbance

%% TFR Loop
simTime = 0;
Tf = 150;
u = [100; 0];

TFRptCloudWorld = [];
TracePose = [];


while simTime < Tf

    hAircraft.move(u,modelF);

    % TFRadar scan
    hTFRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    hTFRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hTFRadar.scanTFRmode;

    TFRptCloud = hTFRadar.TFRptCloudWorld.Location;
    TFRptCloud = TFRptCloud(~isnan(TFRptCloud(:,1)),:);
    TracePose = [TracePose, hAircraft.Pose]; 
    TFRptCloudWorld = [TFRptCloudWorld ; TFRptCloud];
    TFRptRelUAV =  TFRptCloud - hAircraft.Pose(1:3,:)';
    TFRptRelUAV = [sqrt(sum(TFRptRelUAV.^2,2)) , TFRptCloud(:,3)];

    interp_x = linspace(min(TFRptRelUAV(:,1)),max(TFRptRelUAV(:,1)),200);
    interp_y = interp1(TFRptRelUAV(:,1),TFRptRelUAV(:,2),interp_x,"spline");


    % plot(TFRptRelUAV(:,1),TFRptRelUAV(:,2),'g.')
    plot(interp_x,interp_y,'g.')
    xlim([0 11000])
    ylim([0 500])
    clf(gca)
    set(gca,'Color','k')
    

    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow
    simTime = simTime + Ts;     
    simTime
end

%% Plotting

figure(1)
plot3(TFRptCloudWorld(:,1),TFRptCloudWorld(:,2),TFRptCloudWorld(:,3),'*')
daspect([2 1 1])

figure(2)
%distance = 


