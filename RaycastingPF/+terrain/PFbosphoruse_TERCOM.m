% This is a demo, where an aircraft bot flies over the Bosphorus terrain,
% and performs terrain matching.

clc; clear; close all;
addpath(genpath('C:\Users\user\Desktop\githubdeneme\pointssim'))
rng(5,'twister')
%% Create simulation objects
scene = 1;
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
    left_lower = [41 29];
    right_upper = [41.30 29.20];
    hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
    modelF = 1;
elseif scene == 2
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

% Aircraft settings
% Initial pose
% load("data/topgun_newtraj.mat")
% load("data/topgun_traj_heading.mat")
% load("data/topgun_traj_velocity.mat")

% topgun_traj_velocity(1) = 1000;
% topgun_traj_heading(1) = pi/2;

% x0      = topgun_pos(1,1);
% y0      = topgun_pos(1,2);
% z0      = 1000;
% x0      = 1.1324e+04;
% y0      = 3.9600e+03;
% z0      = 1000;
%psi0    = 0*20*pi/180;
% 
% psi0    = topgun_traj_heading(1);

x0      = 1500;
y0      = 1500;
z0      = 500;
psi0    = 20*pi/180;

Ts      = 0.01;
hAircraft.Pose          = [x0; y0; z0; psi0];
hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
hAircraft.dt            = Ts;
hAircraft.WithNoise     = false;      % Enables wind disturbance


TracePose = [hAircraft.Pose];
TraceEstimatedPose = [];
%TraceEstimatedPose_slid = [];
var = [];
%var_slid = [];

% Estimator settings
iPart = 100;
N = 200;
range_part = 500;
alt_std = 3;
raycast_flag = false;
batch_size = 10;

%hEstimator = terrain.StateEstimatorPF(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts,batch_size);
%hEstimator = terrain.StateEstimatorTERCOM(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts,batch_size);
hEstimator.hReferenceMapScanner = hReferenceMapScanner;

% hEstimator_slid_COR = terrain.StateEstimatorPFcor(N,hAircraft.Pose,range_part,range_part,0,alt_std,Ts);
% hEstimator_slid_COR.hReferenceMapScanner = hReferenceMapScanner;

%% Game Loop
%r = rateControl(1/Ts);
%reset(r);
simTime = 0;
Tf = 10;
loop_sampling = Tf/Ts;
%topgun_traj_headdot = [diff(topgun_traj_heading) ; 0];
% u = [topgun_traj_velocity(1:length(topgun_traj_velocity)/loop_sampling:end)...
%      topgun_traj_heading(1:length(topgun_traj_heading)/loop_sampling:end)];
u = [100; 2*pi/500];

i = 1;
Tf = 150;
particles_history(1:N,:) = hEstimator.particles(:,1:2);
%particles_history_slid(1:N,:) = hEstimator_slid_COR.particles(:,1:2);

% hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
% %hRadar.positionLiDAR    =  [18920.5565878799 ; 5600.82778773042 ; z0];
% hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
% hRadar.scanTerrain(false);
% first_radar = hRadar.ptCloud.Location;

while simTime < Tf

    %u = [100; 2*pi/500];

    %hAircraft.move(u(i,:),2);
    hAircraft.move(u,modelF);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    %hRadar.positionLiDAR    =  [18920.5565878799 ; 5600.82778773042 ; z0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    %hRadar.scanTerrain;
    hRadar.scanAltimeter;

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    % param1 = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
    %                                 u(i,:),hRadar.ptCloud, false,2);

    param = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u,hRadar.ptCloud, false,modelF);

    % param2 = hEstimator_slid_COR.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
    %                                 u(i,:),hRadar.ptCloud, false,2);
    toc

    radar_pt_sensor = hRadar.ptCloud.Location;
    %radar_pt_sensor = first_radar;
    %radar_pt_sensor(logical(hEstimator_ray.non_idx(iPart,:)'),:) = [];
    radar_pt_world = zeros(size(radar_pt_sensor));

    %particle_pt_sensor_ray = reshape(hEstimator_ray.particles_pc{iPart},size(radar_pt_sensor));
    particle_pt_sensor = hEstimator.particles_pc{iPart};
    particle_pt_world = zeros(size(particle_pt_sensor));

    %particle_pt_sensor_slid = reshape(hEstimator_slid.particles_pc{iPart},size(radar_pt_sensor));
    % particle_pt_sensor_slid = hEstimator_slid_COR.particles_pc{iPart};
    % particle_pt_world_slid = zeros(size(particle_pt_sensor_slid));

    sTOw_radar = terrain.AbstractRayCasting3D.rTs(hRadar.positionLiDAR) * ...
                     terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    sTOw_particle = terrain.AbstractRayCasting3D.rTs(hEstimator.particles(iPart,1:3)') * ...
                 terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    % sTOw_particle_slid = terrain.AbstractRayCasting3D.rTs(hEstimator_slid_COR.particles(iPart,1:3)') * ...
    %          terrain.AbstractRayCasting3D.wRs(hRadar.orientationLiDAR(1),true);

    %radar_pt_world = radar_pt_world(:,1:3);
    for q=1:length(radar_pt_sensor(:,1))
        array = sTOw_radar * [radar_pt_sensor(q,:)'; 1];
        radar_pt_world(q,1:3) = array(1:3);
    end

    for q=1:length(particle_pt_sensor(:,1))
        array = sTOw_particle * [particle_pt_sensor(q,:)'; 1];
        particle_pt_world(q,1:3) = array(1:3);
    end

    % for q=1:length(particle_pt_sensor_slid(:,1))
    %     array = sTOw_particle_slid * [particle_pt_sensor_slid(q,:)'; 1];
    %     particle_pt_world_slid(q,1:3) = array(1:3);
    % end

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

    %disp([hEstimator_ray.mean_sqrd_error(iPart) ,hEstimator_slid.mean_sqrd_error(iPart) ]);
    %disp(['PF Point Cloud point-wise averaged error: ',num2str(mean(hEstimator.mean_sqrd_error))])
    %disp(['PF-Slid PC averaged error: ',num2str(mean(hEstimator_slid_COR.mean_sqrd_error))])

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
valid_index = ~isnan(TraceEstimatedPose);
%valid_index_slid = ~isnan(TraceEstimatedPose_slid);


TracePose = TracePose(1:2,:);
diff = reshape(TracePose(logical([[0; 0] valid_index])),2,[]) - reshape(TraceEstimatedPose(valid_index),2,[]);
%diff_slid = reshape(TracePose(logical([[0; 0] valid_index_slid])),2,[]) - reshape(TraceEstimatedPose_slid(valid_index_slid),2,[]);

mean_error = mean(sqrt(diff(1,:).^2 + diff(2,:).^2));
%mean_error_slid = mean(sqrt(diff_slid(1,:).^2 + diff_slid(2,:).^2));

mean_std = mean(sqrt(var));
%mean_std_slid = mean(sqrt(var_slid));

disp(['Estimation error of PF ',num2str(mean_error),' meters mean and ',num2str(mean_std),' std'])
%disp(['Estimation error of PF-Slid ',num2str(mean_error_slid),' meters mean and ',num2str(mean_std_slid),' std'])
% 
% ru = [36.2456 -112.323];
% ll = [36.2083 -112.351];
% ru = [36.2369 -112.405];  nan location
% ll = [36.2161 -112.424];
ll = [41.01 29.01];
ru = [41.025 29.03];
hDEM.visualizeDTED(ll,ru); hold on;
%hDEM.visualizeDTED(left_lower,right_upper); hold on;
view(0,90)

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

%p1 = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'b.');
p1 = plot3(particle_lla(:,2),particle_lla(:,1),particle_lla(:,3),'b.');
%p2 = plot3(particles_lla_slid(:,2),particles_lla_slid(:,1),particles_lla_slid(:,3),'b.');
p3 = plot3(TracePose_lla(:,2),TracePose_lla(:,1),TracePose_lla(:,3),'g.');
%p4 = plot3(TraceEstimatedPose_lla(:,2),TraceEstimatedPose_lla(:,1),TraceEstimatedPose_lla(:,3),'r+');
%radar_pt_p = plot3(radar_pt_lla(:,2),radar_pt_lla(:,1),radar_pt_lla(:,3),'c.');
%particle_pt_p_ray = plot3(particle_pt_lla(:,2),particle_pt_lla(:,1),particle_pt_lla(:,3),'m.');
%particle_pt_p_slid = plot3(particle_pt_lla_slid(:,2),particle_pt_lla_slid(:,1),particle_pt_lla_slid(:,3),'y.');

% p1.MarkerSize = 5;
%p2.MarkerSize = 1;

legend({'DTED Mesh','Particles','True Position','PF Estimation','Radar PC','PF Radar PC'},Location="best")

