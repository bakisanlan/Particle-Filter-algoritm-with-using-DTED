% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clc; clear; close all;

rng(5,'twister')
%% Create simulation objects
scene = 2;
hDEM                    = terrain.DigitalElevationModel(scene);
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
% bosphorus
if scene == 1
    hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
elseif scene == 2
    left_lower = [36.18777778 -112.54111111];
    right_upper = [36.38000000 -112.31166667];
    hRadar.DTED = hDEM.getMetricGridElevationMap(left_lower,right_upper, 10);
    hRadar.positionLiDAR(3) = 3000;
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

% Aircraft settings
% Initial pose
load("data/topgun_newtraj.mat")
load("data/topgun_traj_heading.mat")
load("data/topgun_traj_velocity.mat")

topgun_traj_velocity(1) = 1000;
topgun_traj_heading(1) = pi/2;

x0      = topgun_pos(1,1);
y0      = topgun_pos(1,2);
z0      = 1000;
%psi0    = 0*20*pi/180;
psi0    = topgun_traj_heading(1);

Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;

TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];

% Estimator settings
N =200;
hEstimator = terrain.StateEstimatorPF(N,hAircraft.Pose,500,500,0,10,Ts);
hEstimator.hReferenceMapScanner = hReferenceMapScanner;

%% Game Loop
%r = rateControl(1/Ts);
%reset(r);
simTime = 0;
Tf = 200;
loop_sampling = Tf/Ts;
%topgun_traj_headdot = [diff(topgun_traj_heading) ; 0];
u = [topgun_traj_velocity(1:length(topgun_traj_velocity)/loop_sampling:end)...
     topgun_traj_heading(1:length(topgun_traj_heading)/loop_sampling:end)];
i = 1;

Tf = 40;
while simTime < Tf

    %u = [100; 2*pi/500];
    particles_history(1+N*(i-1):N*i,:) = hEstimator.particles(:,1:2);

    hAircraft.move(u(i,:),2);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain(false);

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    param = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u(i,:),hRadar.ptCloud, false,2);
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
    i = i+1;
end
%%
figure(1); hold on;
plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',5);
plot(particles_history(:,1),particles_history(:,2),'k.','MarkerSize',1)
legend('Truth', 'Estimated', 'Particles');
valid_index = ~isnan(TraceEstimatedPose);

TracePose = TracePose(1:2,:);
diff = reshape(TracePose(logical([[0; 0] valid_index])),2,[]) - reshape(TraceEstimatedPose(valid_index),2,[]);
mean_error = mean(sqrt(diff(1,:).^2 + diff(2,:).^2));
disp(['Average error is ',num2str(mean_error),' meters'])

hDEM.visualizeDTED(left_lower,right_upper); hold on;
particles_lla = ned2lla([particles_history(:,2) particles_history(:,1) -z0*ones(length(particles_history(:,1)),1)],[left_lower 0],"flat");
TracePose_lla = ned2lla([TracePose(2,:)' TracePose(1,:)' -z0*ones(length(TracePose(1,:)),1)],[left_lower 0],"flat");
TraceEstimatedPose_lla = ned2lla([TraceEstimatedPose(2,:)' TraceEstimatedPose(1,:)' -z0*ones(length(TraceEstimatedPose(1,:)),1)],[left_lower 0],"flat");

p1 = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'k.');
p2 = plot3(TracePose_lla(:,2),TracePose_lla(:,1),TracePose_lla(:,3),'g*');
p3 = plot3(TraceEstimatedPose_lla(:,2),TraceEstimatedPose_lla(:,1),TraceEstimatedPose_lla(:,3),'r+');

p1.MarkerSize = 10;
