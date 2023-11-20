clc;clear;close all;

%% SIMULATION PARAMETERS
tic

% aircraft states and inputs
alt = 400; % constant altitude of aircraft
velocity = 270; % m/s
heading = deg2rad(0); %rad
head_inc = 0;
u = [heading ; velocity]; % input vector
gps_lost_pos = [5000 ; 5000]; % x and y position when GPS lost
aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos

% sensor property
% 0 or close to 0 altitude error cause to particles gets very very lower
% probability from pdf so resampling will not working very well
alt_std = 3;  % altimeter sensor std error valu
% IMU error provide exploration of particles rather than exploit roughly 
imu_std = [deg2rad(1) 10];  % imu sensor u1 = heading,  u2 = velocity


% particles property
N = 500; % number of particles
range_part = 3000; % initial particles range among aircraft
x_range = [gps_lost_pos(1) - 0.5*range_part ; gps_lost_pos(1) + 0.5*range_part];
y_range = [gps_lost_pos(2) - 0.5*range_part ; gps_lost_pos(2) + 0.5*range_part];
hdg_range = [0 2*pi];

% simulation parameters
dt = 1;
step = 50;

% creating historical array for plotting purposes
real_pos = zeros(step,2);
estimated_pos = zeros(step,2);
particles_history = zeros(step*N,2);

% 
PF = ParticleFilter(N,x_range,y_range,hdg_range,alt_std,imu_std,dt,alt);

% Finding left lower and right upper corner of DTED maps for interpolation
% and plotting
for j= 1:step
    real_pos(j,:) = aircraft_pos(1:2);
    aircraft_pos = PF.aircraft_step(aircraft_pos,u);
    u(1) = u(1) + deg2rad(head_inc);
end

% reset aircraft_pos values to initial values after collecting real_pos
aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos

% finding baundary of DTED map based on aircraft motion
ref_lla = [41.10071636898535, 29.024554581047795];
x_max_vehic = max(real_pos(:,1));
y_max_vehic = max(real_pos(:,2));
x_min_vehic = min(real_pos(:,1));
y_min_vehic = min(real_pos(:,2));
emapf = 5000;                 %expanding map in metrees to N and E direction for visulization
boundary_right_upper_lla = ned2lla([x_max_vehic+emapf y_max_vehic+emapf 0], [ref_lla 0],'flat');
boundary_left_lower_lla = ned2lla(-[x_min_vehic-emapf y_min_vehic-emapf 0], [ref_lla 0],'flat');

% creating DTED value for interpolation step
h1 = DigitalElevationModel;
ndownsample = 4;
ef = 1;  % expanding DTED area with a factor for preventing NaN values
dted = h1.getMetricGridElevationMap(boundary_left_lower_lla-ef, boundary_right_upper_lla+ef, ndownsample);


% initilizating estimation values
[mean, var] = PF.estimate();

% resetting heading input
u(1) = deg2rad(0); %rad

% figure for seeing estimation process of particles step by step
fig = figure(1);

%% Simulation of PF
for i=1:step

    % Plotting and all other things for visulization
    %real_pos(i,:) = aircraft_pos(1:2);
    estimated_pos(i,:) = mean;
    particles_history(1+N*(i-1):N*i,:) = PF.particles(:,1:2);

    %calculating error between estimated and real pos
    estim_error = norm(mean' - aircraft_pos(1:2));

    % plot of estimation process of particles step by step
    p = plot(PF.particles(:,2),PF.particles(:,1),'y.', ...
             aircraft_pos(2),aircraft_pos(1),'r+', ...
             mean(2),mean(1),'*b');
    p(1).MarkerSize = 1;
    p(2).MarkerSize = 5;
    p(3).MarkerSize = 5;
    legend('Particles','True Position','PF Estimation')
    xlim([aircraft_pos(2)-3000 aircraft_pos(2)+3000]);
    ylim([aircraft_pos(1)-3000 aircraft_pos(1)+3000]);
    xlabel('East(m)')
    ylabel('North(m)')
    title('One Step Particles Aircraft Motion and Particles')
    grid on
    % adding text that shows error
    text(aircraft_pos(2)-2500,aircraft_pos(1)+2500,['Distance Error = ' num2str(estim_error)],'Color','red','FontSize',10)

    % pause simulation for seeing clearly step by step
    %pause(0.1);
    clf(fig)

    % PARTICLE FILTER ALGORITHM

    %  move aircraft for taking real radar value
    aircraft_pos = PF.aircraft_step(aircraft_pos,u);
    radar_data = interp2(dted{1},dted{2},dted{3},aircraft_pos(1),aircraft_pos(2));
    radar_data = alt - radar_data + randn(1)*alt_std; % adding altimeter sensor error

    % move particles for Particle Filter algorithm and finding elevation
    % with DTED
    PF.particle_step(u);

    % updating weights of particles
    PF.update_weights(radar_data,dted)

    % taking mean and var variable for estimation metric
    [mean, var] = PF.estimate();

    % change input for seeing different case
    u(1) = u(1) + deg2rad(head_inc);

end

%% Plotting Sim Results
close(fig)

% 3D Figure of DTED map,particles and estimation figure
h1.visualizeDTED(ref_lla,boundary_right_upper_lla);
hold on 
particles_lla = ned2lla([particles_history(:,1) particles_history(:,2) -alt*ones(length(particles_history(:,1)),1)],[ref_lla 0],'flat');
real_pos_lla = ned2lla([real_pos(:,1) real_pos(:,2) -alt*ones(length(real_pos(:,1)),1)],[ref_lla 0],'flat');
estimated_pos_lla = ned2lla([estimated_pos(:,1) estimated_pos(:,2) -alt*ones(length(estimated_pos(:,1)),1)],[ref_lla 0],'flat');

p = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'y.', ...
          real_pos_lla(:,2),real_pos_lla(:,1),real_pos_lla(:,3),'r+'   , ...
          estimated_pos_lla(:,2),estimated_pos_lla(:,1),estimated_pos_lla(:,3),'*b');
p(1).MarkerSize = 1;
p(2).MarkerSize = 5;
p(3).MarkerSize = 5;
legend({'DTED Mesh','Particles','True Position','PF Estimation'},Location="best")
title('Particles in 3D DTED Map','FontSize',20)
grid on
% adjusting aspect ratio of X and Y data equally spaced
cur_aspect = daspect;
daspect([1 1 10000]);
set(gca,'BoxStyle','full','Box','on')

% 2D Figure of particles and estimation figure
figure(2);
p = plot(particles_history(:,2),particles_history(:,1),'y.', ...
                           real_pos(:,2),real_pos(:,1),'r+', ...
                           estimated_pos(:,2),estimated_pos(:,1),'*b');
p(1).MarkerSize = 1;
p(2).MarkerSize = 5;
p(3).MarkerSize = 5;
legend('Particles','True Position','PF Estimation')
xlabel('East(m)')
ylabel('North(m)')
title('Particles in 2D Map','FontSize',20)
grid on
axis equal

toc
