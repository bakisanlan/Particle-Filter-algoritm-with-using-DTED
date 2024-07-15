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
hRadar.dtheta   = 10;
hRadar.dpsi     = 10;
hRadar.rayRange = 1500;

% Simulation Settings
dt = 2;
tf = 200;

if scene == 'BP'
    left_lower = [41 29];
    right_upper = [41.30 29.20];
    hRadar.DTED = hDEM.getMetricGridElevationMap([41 29],[41.30 29.20], 10);
    modelF = 1;

    x0      = 1500;
    y0      = 1500;
    z0      = 300;
    psi0    = 20*pi/180;
    u = repmat([100 2*pi/500],100,1);
elseif scene == 'GC'
    left_lower = [36.18777778 -112.54111111];
    right_upper = [36.38000000 -112.31166667];
    hRadar.DTED = hDEM.getMetricGridElevationMap(left_lower,right_upper, 10);
    hRadar.positionLiDAR(3) = 3000;
    modelF = 2;

    load(fullfile(fileparts(mfilename('fullpath')),'data/topgun_newtraj.mat'),'topgun_pos')
    load(fullfile(fileparts(mfilename('fullpath')),'data/topgun_traj_heading.mat'),'topgun_traj_heading')
    load(fullfile(fileparts(mfilename('fullpath')),'data/topgun_traj_velocity.mat'),'topgun_traj_velocity')

    topgun_traj_velocity(1) = 1000;
    topgun_traj_heading(1) = pi/2;
    x0      = topgun_pos(1,1);
    y0      = topgun_pos(1,2);
    z0      = 1500;
    psi0    = topgun_traj_heading(1); %rad

    aircraft_model_flag = 2;  % Kinematic model
    loop_sampling = tf/dt;
    % selecting u inputs for Grand Canyon flight scenario
    u = [topgun_traj_velocity(1:length(topgun_traj_velocity)/loop_sampling:end)...
        topgun_traj_heading(1:length(topgun_traj_heading)/loop_sampling:end)];
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
%% Reference map scanner settings
hReferenceMapScanner.dtheta     = hRadar.dtheta;
hReferenceMapScanner.dpsi       = hRadar.dpsi;
hReferenceMapScanner.rayRange   = hRadar.rayRange;
hReferenceMapScanner.DTED       = hRadar.DTED;

%% Aircraft setting
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

%% Estimator settings
iPart = 100;
N = 100;
range_part = 500;
alt_std = 3;
raycast_flag = false;
batch_size = 1;
exp_rate = 0;
hEstimator = terrain.StateEstimatorPF(N,hAircraft.Pose,range_part,range_part,exp_rate,alt_std,dt,batch_size);
hEstimator.hReferenceMapScanner = hReferenceMapScanner;
particles_history(1:N,:) = hEstimator.particles(:,1:2);

%% Game Loop
%r = rateControl(1/dt);
%reset(r);
simTime = 0;
i = 1;

while simTime < tf
    hAircraft.move(u(i,:),modelF);

    % Radar scan
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; phi_r];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    %hRadar.scanTerrain;
    hRadar.scanAltimeter;
    hReferenceMapScanner.flagScanAltimeter = hRadar.flagScanAltimeter;

    % Estimate using x-y grid overlaying and not full ray casting
    tic
    param = hEstimator.getEstimate([hRadar.positionLiDAR; hRadar.orientationLiDAR], ...
                                    u(i,:),hRadar.ptCloud, raycast_flag, modelF);
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

    % 2D Live graph update
    hLocationPoint.XData = hAircraft.Pose(1);
    hLocationPoint.YData = hAircraft.Pose(2);
    hLocationPoint.ZData = hAircraft.Pose(3);
    snapnow

    %waitfor(r);

    % Store the specific particle and all particles for plotting purpose
    particles_history(1+N*(i):N*(i+1),:) = hEstimator.particles(:,1:2);
    particle_history(i,:) = hEstimator.particles(iPart,1:2);
    
    % Update simulation time
    simTime = simTime + dt;
    i = i+1;

end
%% Display mean and std of error of estimation
valid_index = ~isnan(TraceEstimatedPose);
TracePose = TracePose(1:2,:);
diff = reshape(TracePose(logical([[0; 0] valid_index])),2,[]) - reshape(TraceEstimatedPose(valid_index),2,[]);
errors = sqrt(diff(1,:).^2 + diff(2,:).^2);
mean_error = mean(errors);

mean_std = sqrt(sum((errors - mean_error).^2)/N);

disp(['Estimation error of PF ',num2str(mean_error),' meters mean and ',num2str(mean_std),' std'])

%% 2D Graph of Ground Truth, Estimation and Particles
figure(2); hold on;
plot(particles_history(:,1),particles_history(:,2),'k.','MarkerSize',1)
plot(TracePose(1,:),TracePose(2,:),'b:o','MarkerSize',10);
plot(TraceEstimatedPose(1,:),TraceEstimatedPose(2,:),'rx','MarkerSize',10);
daspect([1 1 1])
pbaspect([1 1 1])
legend('Truth', 'Estimated','Particles');
%% 3D Graph of Estimation
% ll = [41.012 29.015];
% ru = [41.02 29.030];
%hDEM.visualizeDTED(ll,ru); hold on;
hDEM.visualizeDTED(left_lower,right_upper); hold on;
%view(0,90)

particles_lla = ned2lla([particles_history(:,2) particles_history(:,1) -z0*ones(length(particles_history(:,1)),1)],[left_lower 0],"flat");
particle_lla = ned2lla([particle_history(:,2) particle_history(:,1) -z0*ones(length(particle_history(:,1)),1)],[left_lower 0],"flat");
TracePose_lla = ned2lla([TracePose(2,:)' TracePose(1,:)' -z0*ones(length(TracePose(1,:)),1)],[left_lower 0],"flat");
TraceEstimatedPose_lla = ned2lla([TraceEstimatedPose(2,:)' TraceEstimatedPose(1,:)' -z0*ones(length(TraceEstimatedPose(1,:)),1)],[left_lower 0],"flat");
radar_pt_lla =  ned2lla([radar_pt_world(:,2) radar_pt_world(:,1) -radar_pt_world(:,3)],[left_lower 0],"flat");
particle_pt_lla =  ned2lla([particle_pt_world(:,2) particle_pt_world(:,1) -particle_pt_world(:,3)],[left_lower 0],"flat");

p1 = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'b.');
p3 = plot3(TracePose_lla(:,2),TracePose_lla(:,1),TracePose_lla(:,3),'g.');
p4 = plot3(TraceEstimatedPose_lla(:,2),TraceEstimatedPose_lla(:,1),TraceEstimatedPose_lla(:,3),'r+');
radar_pt_p = plot3(radar_pt_lla(:,2),radar_pt_lla(:,1),radar_pt_lla(:,3),'c.');
particle_pt_p = plot3(particle_pt_lla(:,2),particle_pt_lla(:,1),particle_pt_lla(:,3),'m.');

% p1.MarkerSize = 5;
%p2.MarkerSize = 1;

legend({'DTED Mesh','Particles','True Position','PF Estimation','Radar PC','PF Radar PC'},Location="best")
