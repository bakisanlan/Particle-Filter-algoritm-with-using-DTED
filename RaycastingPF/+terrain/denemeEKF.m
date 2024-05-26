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
hAircraftINS            = terrain.AircraftBot(zeros(4,1),0);

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
    DTED = hRadar.DTED;
    modelF = 1;
elseif scene == 2
    left_lower = [36.18777778 -112.54111111];
    right_upper = [36.38000000 -112.31166667];
    hRadar.DTED = hDEM.getMetricGridElevationMap(left_lower,right_upper, 10);
    hRadar.positionLiDAR(3) = 3000;
    modelF = 2;
end

% % Gameloop plot
% hRadar.showMap; axis('auto');
% hRadar.hFigure.CurrentAxes.Children(2).FaceColor = 'flat'; view(2);
% colorbar;
% hLocationPoint = hRadar.hFigure.Children(3).Children(1); % handle to aircraft location
% hLocationPoint.Marker = "^";
% hLocationPoint.MarkerSize = 10;
% hLocationPoint.MarkerFaceColor = "cyan";


% Initial Condition of UAV
x0      = 1500;
y0      = 1500;
z0      = 500;
psi0    = 20*pi/180;
Ts = 0.1;

hAircraft.Pose          = [x0; y0; z0; psi0];
hAircraft.dt            = Ts;
hAircraft.WithNoise     = false;      % Enables wind disturbance
hAircraftINS.Pose          = [x0; y0; z0; psi0];
hAircraftINS.dt            = Ts;
hAircraftINS.WithNoise     = true;      % Enables wind disturbance


% creating of TERCOM grid
xy_lim = 300;
grid_n = 30;
x = linspace(x0-xy_lim/2,x0+xy_lim/2,grid_n+1);
y = linspace(y0-xy_lim/2,y0+xy_lim/2,grid_n+1);
tercom_grid = zeros(length(x)*length(y),3);
batch_size = 10;
batch_n = 1;
batch_tercom_grid_z_error = zeros(length(tercom_grid(:,1)),batch_size);
row = 1;
for i=1:length(x)
    for j=1:length(y)
        tercom_grid(row,:) = [x(i) y(j) 0];
        row = row +1 ;
    end
end

% creating plot array
TracePose = [];
TraceINS = [];
TraceEstimatedPose = [];
%TraceEstimatedPose_slid = [];
var = [];

% defining inputs
u = [100; 0];

% Defining EKF matrix
% Initilize P state covariance matrix
P = diag([0 0 0 deg2rad(1^2)]);

% Create Q Process covariance matrix
Q = diag([(hAircraft.Sigma).^2 ; 0 ; 0]);

% Create R measurement covariance matrix
R = 3^2;


% hAircraft.move(u,modelF)
% tercom_grid(:,1:2) = tercom_grid(:,1:2) + hAircraft.dx(1:2).';
% tercom_grid(:,3) = interp2(DTED{1},DTED{2},DTED{3},tercom_grid(:,2),tercom_grid(:,1));

% Game loop
simTime = 0;
Tf =100;
c = 1;

while simTime < Tf


    hAircraft.move(u,modelF);
    hAircraftINS.move(u,modelF);

    % Radar scan from real aircraft position
    hRadar.orientationLiDAR = [hAircraft.Pose(4)*180/pi; 0; 0];
    hRadar.positionLiDAR    =  hAircraft.Pose(1:3);
    hRadar.scanAltimeter;

    % TERCOM grid update
    tercom_grid(:,1:2) = tercom_grid(:,1:2) + hAircraft.dx(1:2).';
    tercom_grid(:,3) = interp2(DTED{1},DTED{2},DTED{3},tercom_grid(:,1),tercom_grid(:,2));

    % Batch update
    if batch_n == batch_size+1
        batch_tercom_grid_z_error(:,1) = [];
        batch_tercom_grid_z_error(:,batch_n-1) = abs(tercom_grid(:,3) - hRadar.zTERCOM);

        [lowest_MAD , idx] = min(mean(batch_tercom_grid_z_error(:,:),2));
        CR_DR = tercom_grid(idx,1:2); % [cross range, down range] = TERCOM grid best match position
        idx

        % Slope calculation at CR,DR pos
        %TERCOM_grid_surf = surf(x,y,reshape(tercom_grid(:,3),[length(y) , length(x)]));
        TERCOM_z_grid = reshape(tercom_grid(:,3),[length(y) , length(x)]);
        xi = fix(idx/length(y)) + 1;
        yi = rem(idx,length(y));
        if yi == 0
            yi = length(y);
        end

        if xi > length(x)
            xi = length(x);
        end

        dCR = (1/(2*x(2)-x(1))) * (TERCOM_z_grid(yi,min(length(x),xi+1)) - TERCOM_z_grid(yi,max(1,xi-1)));
        dDR = (1/(2*y(2)-y(1))) * (TERCOM_z_grid(min(length(y),yi+1),xi) - TERCOM_z_grid(max(1,yi-1),xi));

        % local slope = linearized measurement function of kalman filter H
        H = [dCR dDR 0 0];
        z_pred = tercom_grid(idx,3);

        % F matrix
        % Jocabian of state transition matrix F
        F = [1 0 0  -u(1)*sin(hAircraft.Pose(4))*Ts ;
             0 1 0   u(1)*cos(hAircraft.Pose(4))*Ts ;
             0 0 1   0                              ;
             0 0 0   1                              ];

        % Kalman equations
        P = F * P * F.' + Q;
        residual = lowest_MAD;
        K = P * H.' * inv(H * P * H.' + R);
        P = (eye(length(H)) - K * H) * P;% * inv(eye(length(H)));% - K * H).' + K * R * K.';
        x_est = [CR_DR 0 0].' + K*residual;
        x_est = x_est(1:2);
        TracePose = [TracePose ; hAircraft.Pose(1:2).'];
%        TraceEstimatedPose = [TraceEstimatedPose ; x_est.'];
        TraceEstimatedPose = [TraceEstimatedPose ; CR_DR];



        % Game plot
        fig = figure(1); hold on;
        plot(TracePose(c,1), TracePose(c,2),'g*');
        plot(TraceEstimatedPose(c,1), TraceEstimatedPose(c,2),'r+');
        plot(tercom_grid(:,1), tercom_grid(:,2),'k.',MarkerSize=1);
        xlim([1500 12000])
        ylim([1500 5000])
        pause(0.1)
        cla(gca)

        c = c +1;
    else
        batch_tercom_grid_z_error(:,batch_n) = abs(tercom_grid(:,3) - hRadar.zTERCOM);
        batch_n = batch_n + 1;
    end

    simTime = simTime + Ts;

end