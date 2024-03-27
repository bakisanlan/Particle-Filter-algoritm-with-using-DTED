% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clc; clear; close all;
addpath(genpath('C:\Users\user\Desktop\githubdeneme\pointssim'))
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
% x0      = 1.1324e+04;
% y0      = 3.9600e+03;
% z0      = 1000;
%psi0    = 0*20*pi/180;
psi0    = topgun_traj_heading(1);

Ts      = 2;
hAircraft.Pose          = [x0; y0; z0; psi0];
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;

TracePose = [hAircraft.Pose];
TraceEstimatedPose_ray = [];
TraceEstimatedPose_slid = [];
var_ray = [];
var_slid = [];



% Estimator settings
iPart = 100;
N = 100;
range_part = 500;
alt_std = 10;
raycast_flag = false;
hEstimator_slid_PW = terrain.StateEstimatorPF(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts);
hEstimator_slid_PW.hReferenceMapScanner = hReferenceMapScanner;

hEstimator_slid_COR = terrain.StateEstimatorPFcor(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts);
hEstimator_slid_COR.hReferenceMapScanner = hReferenceMapScanner;

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

Tf = 5;
particles_history_ray(1:N,:) = hEstimator_slid_PW.particles(:,1:2);
particles_history_slid(1:N,:) = hEstimator_slid_COR.particles(:,1:2);

% hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
% %hRadar.positionLiDAR    =  [18920.5565878799 ; 5600.82778773042 ; z0];
% hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
% hRadar.scanTerrain(false);
% first_radar = hRadar.ptCloud.Location;

while simTime < Tf

    %u = [100; 2*pi/500];

    hAircraft.move(u(i,:),2);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    %hRadar.positionLiDAR    =  [18920.5565878799 ; 5600.82778773042 ; z0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanTerrain(false);

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    param1 = hEstimator_slid_PW.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u(i,:),hRadar.ptCloud, false,2);

    param2 = hEstimator_slid_COR.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u(i,:),hRadar.ptCloud, false,2);
    toc

    radar_pt_sensor = hRadar.ptCloud.Location;
    %radar_pt_sensor = first_radar;
    %radar_pt_sensor(logical(hEstimator_ray.non_idx(iPart,:)'),:) = [];
    radar_pt_world = zeros(size(radar_pt_sensor));

    %particle_pt_sensor_ray = reshape(hEstimator_ray.particles_pc{iPart},size(radar_pt_sensor));
    particle_pt_sensor_ray = hEstimator_slid_PW.particles_pc{iPart};
    particle_pt_world_ray = zeros(size(particle_pt_sensor_ray));

    %particle_pt_sensor_slid = reshape(hEstimator_slid.particles_pc{iPart},size(radar_pt_sensor));
    particle_pt_sensor_slid = hEstimator_slid_COR.particles_pc{iPart};

    particle_pt_world_slid = zeros(size(particle_pt_sensor_slid));

    sTOw_radar = terrain.AbstractRayCasting3D.rTs(hRadar.positionLiDAR) * ...
                     terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    sTOw_particle_ray = terrain.AbstractRayCasting3D.rTs(hEstimator_slid_PW.particles(iPart,1:3)') * ...
                 terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    sTOw_particle_slid = terrain.AbstractRayCasting3D.rTs(hEstimator_slid_COR.particles(iPart,1:3)') * ...
             terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    %radar_pt_world = radar_pt_world(:,1:3);
    for q=1:length(radar_pt_sensor(:,1))
        array = sTOw_radar * [radar_pt_sensor(q,:)'; 1];
        radar_pt_world(q,1:3) = array(1:3);
    end

    for q=1:length(particle_pt_sensor_ray(:,1))
        array = sTOw_particle_ray * [particle_pt_sensor_ray(q,:)'; 1];
        particle_pt_world_ray(q,1:3) = array(1:3);
    end

    for q=1:length(particle_pt_sensor_slid(:,1))
        array = sTOw_particle_slid * [particle_pt_sensor_slid(q,:)'; 1];
        particle_pt_world_slid(q,1:3) = array(1:3);
    end

    hAircraft.EstimatedPose = [param1(1); param1(2)];
    EstimatedPose_slid = [param2(1); param2(2)];

    %disp (['Estimated [x,y]: ' '[ ' num2str(hAircraft.EstimatedPose') ' ]']);

    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow

    %waitfor(r);

    TracePose = [TracePose, hAircraft.Pose]; %#ok<*AGROW>
    TraceEstimatedPose_ray = [TraceEstimatedPose_ray, hAircraft.EstimatedPose];
    TraceEstimatedPose_slid = [TraceEstimatedPose_slid, EstimatedPose_slid];

    var_ray = [var_ray , hEstimator_slid_PW.var];
    var_slid = [var_slid , hEstimator_slid_COR.var];

    particles_history_ray(1+N*(i):N*(i+1),:) = hEstimator_slid_PW.particles(:,1:2);
    particles_history_slid(1+N*(i):N*(i+1),:) = hEstimator_slid_COR.particles(:,1:2);
    
    % Update simulation time
    simTime = simTime + Ts;
    i = i+1;

    %disp([hEstimator_ray.mean_sqrd_error(iPart) ,hEstimator_slid.mean_sqrd_error(iPart) ]);
    disp(['PF-Ray PC averaged error: ',num2str(mean(hEstimator_slid_PW.mean_sqrd_error))])
    disp(['PF-Slid PC averaged error: ',num2str(mean(hEstimator_slid_COR.mean_sqrd_error))])

    %disp([hEstimator_ray.weights(iPart) hEstimator_slid.weights(iPart) ]);
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
valid_index_ray = ~isnan(TraceEstimatedPose_ray);
valid_index_slid = ~isnan(TraceEstimatedPose_slid);


TracePose = TracePose(1:2,:);
diff_ray = reshape(TracePose(logical([[0; 0] valid_index_ray])),2,[]) - reshape(TraceEstimatedPose_ray(valid_index_ray),2,[]);
diff_slid = reshape(TracePose(logical([[0; 0] valid_index_slid])),2,[]) - reshape(TraceEstimatedPose_slid(valid_index_slid),2,[]);

mean_error_ray = mean(sqrt(diff_ray(1,:).^2 + diff_ray(2,:).^2));
mean_error_slid = mean(sqrt(diff_slid(1,:).^2 + diff_slid(2,:).^2));

mean_std_ray = mean(sqrt(var_ray));
mean_std_slid = mean(sqrt(var_slid));

disp(['Estimation error of PF-Ray ',num2str(mean_error_ray),' meters mean and ',num2str(mean_std_ray),' std'])
disp(['Estimation error of PF-Slid ',num2str(mean_error_slid),' meters mean and ',num2str(mean_std_slid),' std'])

% ru = [36.2456 -112.323];
% ll = [36.2083 -112.351];
% ru = [36.2369 -112.405];  nan location
% ll = [36.2161 -112.424];
%hDEM.visualizeDTED(ll,ru); hold on;
hDEM.visualizeDTED(left_lower,right_upper); hold on;

particles_lla_ray = ned2lla([particles_history_ray(:,2) particles_history_ray(:,1) -z0*ones(length(particles_history_ray(:,1)),1)],[left_lower 0],"flat");
particles_lla_slid = ned2lla([particles_history_slid(:,2) particles_history_slid(:,1) -z0*ones(length(particles_history_ray(:,1)),1)],[left_lower 0],"flat");

TracePose_lla = ned2lla([TracePose(2,:)' TracePose(1,:)' -z0*ones(length(TracePose(1,:)),1)],[left_lower 0],"flat");
TraceEstimatedPose_lla = ned2lla([TraceEstimatedPose_ray(2,:)' TraceEstimatedPose_ray(1,:)' -z0*ones(length(TraceEstimatedPose_ray(1,:)),1)],[left_lower 0],"flat");
radar_pt_lla =  ned2lla([radar_pt_world(:,2) radar_pt_world(:,1) -radar_pt_world(:,3)],[left_lower 0],"flat");
particle_pt_lla_ray =  ned2lla([particle_pt_world_ray(:,2) particle_pt_world_ray(:,1) -particle_pt_world_ray(:,3)],[left_lower 0],"flat");
particle_pt_lla_slid =  ned2lla([particle_pt_world_slid(:,2) particle_pt_world_slid(:,1) -particle_pt_world_slid(:,3)],[left_lower 0],"flat");

% radar_pt_lla= radar_pt_lla(1,:);
% particle_pt_lla_ray = particle_pt_lla_ray(1,:);
% particle_pt_lla_slid = particle_pt_lla_slid(1,:);

p1 = plot3(particles_lla_ray(:,2),particles_lla_ray(:,1),particles_lla_ray(:,3),'r.');
p2 = plot3(particles_lla_slid(:,2),particles_lla_slid(:,1),particles_lla_slid(:,3),'b.');
p3 = plot3(TracePose_lla(:,2),TracePose_lla(:,1),TracePose_lla(:,3),'g*');
p4 = plot3(TraceEstimatedPose_lla(:,2),TraceEstimatedPose_lla(:,1),TraceEstimatedPose_lla(:,3),'r+');
radar_pt_p = plot3(radar_pt_lla(:,2),radar_pt_lla(:,1),radar_pt_lla(:,3),'c.');
particle_pt_p_ray = plot3(particle_pt_lla_ray(:,2),particle_pt_lla_ray(:,1),particle_pt_lla_ray(:,3),'m.');
particle_pt_p_slid = plot3(particle_pt_lla_slid(:,2),particle_pt_lla_slid(:,1),particle_pt_lla_slid(:,3),'y.');

p1.MarkerSize = 1;
p2.MarkerSize = 1;

legend({'DTED Mesh','ParticlesRay','ParticlesSlid','True Position','PF Estimation','PcTrue','PcRay','PcSlid'},Location="best")