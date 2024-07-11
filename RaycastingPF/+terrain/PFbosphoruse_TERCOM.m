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
% psi0    = 20*pi/180;
psi0    = 20*pi/180;
Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
phi_r = 0;
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;
hAircraft.WithNoise     = false;      % Enables wind disturbance

TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];
%TraceEstimatedPose_slid = [];
var = [];
%var_slid = [];

% Estimator settings
iPart = 1;
N = 100;
range_part = 500;
alt_std = 3;
raycast_flag = false;
batch_size = 1;

%hEstimator = terrain.StateEstimatorPF(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts,batch_size);
hEstimator = terrain.StateEstimatorTERCOM(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts,batch_size);
hEstimator.hReferenceMapScanner = hReferenceMapScanner;

% hEstimator_slid_COR = terrain.StateEstimatorPFcor(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts);
% hEstimator_slid_COR.hReferenceMapScanner = hReferenceMapScanner;

%% Game Loop
%r = rateControl(1/Ts);
%reset(r);
simTime = 0;
Tf = 20;
loop_sampling = Tf/Ts;
% u = [100; 0];
i = 1;
Tf = 200;
particles_history(1:N,:) = hEstimator.particles(:,1:2);
%particles_history_slid(1:N,:) = hEstimator_slid_COR.particles(:,1:2);


while simTime < Tf

    u = [100; 2*pi/500];
    %u = [100; 0];


    %hAircraft.move(u(i,:),2);
    hAircraft.move(u,modelF);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain;
    %hRadar.scanAltimeter;

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    % param1 = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
    %                                 u(i,:),hRadar.ptCloud, false,2);

    param = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u,hRadar.ptCloud, raycast_flag, modelF);

    % param2 = hEstimator_slid_COR.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
    %                                 u(i,:),hRadar.ptCloud, false,2);
    toc

    radar_pt_world = hRadar.ptCloud.w.Location;
    particle_pt_world = hEstimator.particles_pc{iPart};

    if ~isempty(param)

        hAircraft.EstimatedPose = [param(1); param(2)];
        TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
        TraceEstimatedPose = [TraceEstimatedPose, hAircraft.EstimatedPose];
        var = [var , hEstimator.var];
        disp(['PF Point Cloud batch averaged error: ',num2str(mean(hEstimator.MAE_particles))])

    end
    %EstimatedPose_slid = [param2(1); param2(2)];

    %disp (['Estimated [x,y]: ' '[ ' num2str(hAircraft.EstimatedPose') ' ]']);

    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow

    %waitfor(r);

    %TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
    %TraceEstimatedPose = [TraceEstimatedPose, hAircraft.EstimatedPose];
    %TraceEstimatedPose_slid = [TraceEstimatedPose_slid, EstimatedPose_slid];

    %var = [var , hEstimator.var];
    %var_slid = [var_slid , hEstimator_slid_COR.var];

    particles_history(1+N*(i):N*(i+1),:) = hEstimator.particles(:,1:2);
    particle_history(i,:) = hEstimator.particles(iPart,1:2);
    %particles_history_slid(1+N*(i):N*(i+1),:) = hEstimator_slid_COR.particles(:,1:2);
    
    % Update simulation time
    simTime = simTime + Ts;
    i = i+1;

end
%%
% particles_history_ray = particles_history_ray([iPart N+iPart],:);
% particles_history_slid = particles_history_slid([iPart N+iPart],:);

% figure(1); hold on;
% plot(particles_history_ray(:,1),particles_history_ray(:,2),'k.','MarkerSize',1)
% plot(particles_history_slid(:,1),particles_history_slid(:,2),'k.','MarkerSize',1)
% 
% plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
% plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',5);
% legend('Truth', 'Estimated', 'Particles');
valid_index = ~isnan(TraceEstimatedPose);
%valid_index_slid = ~isnan(TraceEstimatedPose_slid);


TracePose = TracePose(1:2,:);
diff = reshape(TracePose(logical([[0; 0] valid_index])),2,[]) - reshape(TraceEstimatedPose(valid_index),2,[]);
%diff_slid = reshape(TracePose(logical([[0; 0] valid_index_slid])),2,[]) - reshape(TraceEstimatedPose_slid(valid_index_slid),2,[]);

errors = sqrt(diff(1,:).^2 + diff(2,:).^2);
mean_error = mean(errors);
%mean_error_slid = mean(sqrt(diff_slid(1,:).^2 + diff_slid(2,:).^2));

%mean_std = mean(sqrt(var));
mean_std = sqrt(sum((errors - mean_error).^2)/N);
%mean_std_slid = mean(sqrt(var_slid));

disp(['Estimation error of PF ',num2str(mean_error),' meters mean and ',num2str(mean_std),' std'])
%disp(['Estimation error of PF-Slid ',num2str(mean_error_slid),' meters mean and ',num2str(mean_std_slid),' std'])
% 
% ru = [36.2456 -112.323];
% ll = [36.2083 -112.351];
% ru = [36.2369 -112.405];  nan location
% ll = [36.2161 -112.424];
% ll = [41 29];
% ru = [41.16 29.11];
ll = [41.012 29.015];
ru = [41.02 29.030];

%hDEM.visualizeDTED(ll,ru); hold on;
hDEM.visualizeDTED(left_lower,right_upper); hold on;
%view(0,90)

particles_lla = ned2lla([particles_history(:,2) particles_history(:,1) -z0*ones(length(particles_history(:,1)),1)],[left_lower 0],"flat");
particle_lla = ned2lla([particle_history(:,2) particle_history(:,1) -z0*ones(length(particle_history(:,1)),1)],[left_lower 0],"flat");

%particles_lla_slid = ned2lla([particles_history_slid(:,2) particles_history_slid(:,1) -z0*ones(length(particles_history_ray(:,1)),1)],[left_lower 0],"flat");

TracePose_lla = ned2lla([TracePose(2,:)' TracePose(1,:)' -z0*ones(length(TracePose(1,:)),1)],[left_lower 0],"flat");
TraceEstimatedPose_lla = ned2lla([TraceEstimatedPose(2,:)' TraceEstimatedPose(1,:)' -z0*ones(length(TraceEstimatedPose(1,:)),1)],[left_lower 0],"flat");
radar_pt_lla =  ned2lla([radar_pt_world(:,2) radar_pt_world(:,1) -radar_pt_world(:,3)],[left_lower 0],"flat");
particle_pt_lla =  ned2lla([particle_pt_world(:,2) particle_pt_world(:,1) -particle_pt_world(:,3)],[left_lower 0],"flat");
%particle_pt_lla_slid =  ned2lla([particle_pt_world_slid(:,2) particle_pt_world_slid(:,1) -particle_pt_world_slid(:,3)],[left_lower 0],"flat");

% radar_pt_lla= radar_pt_lla(1,:);
% particle_pt_lla_ray = particle_pt_lla_ray(1,:);
% particle_pt_lla_slid = particle_pt_lla_slid(1,:);

p1 = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'b.');
%p1 = plot3(particle_lla(:,2),particle_lla(:,1),particle_lla(:,3),'b.');
%p2 = plot3(particles_lla_slid(:,2),particles_lla_slid(:,1),particles_lla_slid(:,3),'b.');
p3 = plot3(TracePose_lla(:,2),TracePose_lla(:,1),TracePose_lla(:,3),'g.');
%p4 = plot3(TraceEstimatedPose_lla(:,2),TraceEstimatedPose_lla(:,1),TraceEstimatedPose_lla(:,3),'r+');
radar_pt_p = plot3(radar_pt_lla(:,2),radar_pt_lla(:,1),radar_pt_lla(:,3),'c.');
particle_pt_p_ray = plot3(particle_pt_lla(:,2),particle_pt_lla(:,1),particle_pt_lla(:,3),'m.');
%particle_pt_p_slid = plot3(particle_pt_lla_slid(:,2),particle_pt_lla_slid(:,1),particle_pt_lla_slid(:,3),'y.');

% p1.MarkerSize = 5;
%p2.MarkerSize = 1;

legend({'DTED Mesh','Particles','True Position','PF Estimation','Radar PC','PF Radar PC'},Location="best")

%%

Z = cell2mat(hEstimator.elev_particles_pc);
radar_Z = cell2mat(hEstimator.radar_Z);
tz = 1:length(radar_Z);
MAE_hist = hEstimator.MAE_particles_hist;
MAE_hist_b1 = hEstimator.MAE_particles_hist_b1;

%plot(tz,radar_Z,'g',tz,Z,'b',tz,MAE_hist,'r',tz,MAE_hist_b1,'c')


figure(3)
subplot(1,2,1)
plot(tz,radar_Z,'g',tz,Z,'b')
subplot(1,2,2)
plot(tz,MAE_hist,'r',tz,MAE_hist_b1,'c')


figure(4)
radar_pt_p = plot3(radar_pt_lla(:,2),radar_pt_lla(:,1),radar_pt_lla(:,3),'c.');
hold on
%   particle_pt_p_ray = plot3(particle_pt_lla(:,2),particle_pt_lla(:,1),particle_pt_lla(:,3),'m.');
