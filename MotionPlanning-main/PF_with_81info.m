clc;clear;close all;
addpath(genpath(cd))

%% SIMULATION PARAMETERS
tic

% aircraft states and inputs

load("Straight_Level_Flight.mat")

sample = 10;

alt = -out.trimmedLogsout.find('xyz_m').Values.Data(1,3); % constant altitude of aircraft
velocity = out.trimmedLogsout.find('Velocity').Values.Data(1); % m/s
heading = out.trimmedLogsout.find('Psi_deg').Values.Data(1); %rad      
head_inc = 0;
u = [heading ; velocity]; % input vector
emapf = 2000;
gps_lost_pos = [emapf ; emapf]; % x and y position when GPS lost
%aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos
aircraft_pos = out.trimmedLogsout.find('xyz_m').Values.Data;  % aircraft init pos
aircraft_pos(:,3) = -aircraft_pos(:,3);   % converting up z to down z
aircraft_pos = aircraft_pos(1:sample:end,:);

% sensor property
% 0 or close to 0 altitude error cause to particles gets very very lower
% probability from pdf so resampling will not working very well
alt_std = 3;  % altimeter sensor std error valu
% IMU error provide exploration of particles rather than exploit roughly 
imu_std = [deg2rad(1) 10];  % imu sensor u1 = heading,  u2 = velocity
radar_data = out.trimmedLogsout.find('allRadarPoint_Body_m').Values.Data;
radar_data = radar_data(:,:,sample+1:sample:end);
%a = radar_data(:,:,1);

% particles property
N = 250; % number of particles
range_part = 500; % initial particles range among aircraft
x_range = [gps_lost_pos(1) - 0.5*range_part ; gps_lost_pos(1) + 0.5*range_part];
y_range = [gps_lost_pos(2) - 0.5*range_part ; gps_lost_pos(2) + 0.5*range_part];
hdg_range = [0 2*pi];

% simulation parameters
%dt = 0.01;
%dt = 1;
%tf = 50;
%step = tf/dt + 1;
step = length(aircraft_pos(:,1));
%dt = tf/(step-1);
dt = norm([aircraft_pos(1,1)-aircraft_pos(2,1), aircraft_pos(1,2)-aircraft_pos(2,2)])/velocity;

% creating historical array for plotting purposes
%real_pos = aircraft_pos;
estimated_pos = zeros(step,2);
particles_history = zeros(step*N,2);
elev_particles_pc_history = zeros(step*N,81);

% 
PF = ParticleFilter_multi(N,x_range,y_range,hdg_range,alt_std,imu_std,dt,alt);

% Finding left lower and right upper corner of DTED maps for interpolation
% and plotting
% for j= 1:step
%     real_pos(j,:) = aircraft_pos(1:2);
%     aircraft_pos = PF.aircraft_step(aircraft_pos,u);
%     u(1) = u(1) + deg2rad(head_inc);
% end

% reset aircraft_pos values to initial values after collecting real_pos
%aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos

% finding baundary of DTED map based on aircraft motion
ref_lla = [41.10071636898535, 29.024554581047795];  % itu arc
x_max_vehic = max(aircraft_pos(:,1));
y_max_vehic = max(aircraft_pos(:,2));
x_min_vehic = min(aircraft_pos(:,1));
y_min_vehic = min(aircraft_pos(:,2));
%emapf = 2000;                 %expanding map in metrees to N and E direction for visulization
boundary_right_upper_lla = ned2lla([x_max_vehic+emapf y_max_vehic+emapf 0], [ref_lla 0],'flat');
boundary_left_lower_lla  = ned2lla([x_min_vehic-emapf y_min_vehic-emapf 0], [ref_lla 0],'flat');

% creating DTED value for interpolation step
h1 = DigitalElevationModel;
ndownsample = 1;
ef = 1;  % expanding DTED area with a factor for preventing NaN values
%dted = h1.getMetricGridElevationMap(boundary_left_lower_lla-ef, boundary_right_upper_lla+ef, ndownsample);

% translation of aircraft pos w.r.t baundary left point distance
correction_translation_loc = lla2ned([ref_lla 0] ,[boundary_left_lower_lla(1:2) 0],"flat");
aircraft_pos(:,1:2) = aircraft_pos(:,1:2) + correction_translation_loc(1:2);

boundary_right_upper_lla = ned2lla(correction_translation_loc , boundary_right_upper_lla,'flat');

dted = h1.getMetricGridElevationMap(boundary_left_lower_lla, boundary_right_upper_lla, ndownsample);

%h1.visualizeDTED(boundary_left_lower_lla,boundary_right_upper_lla)

% initilizating estimation values
[meann, var] = PF.estimate();

% resetting heading input
u(1) = deg2rad(0); %rad

% figure for seeing estimation process of particles step by step
fig = figure(1);

k = 1;
%% Simulation of PF
for i=1:step
    %i;
    if mod(i,round(step/100)) == 0
        disp(k)
        k = k+1;
    end

    % Plotting and all other things for visulization
    %real_pos(i,:) = aircraft_pos(1:2);
    estimated_pos(i,:) = meann;
    particles_history(1+N*(i-1):N*i,:) = PF.particles(:,1:2);

    %calculating error between estimated and real pos
    estim_error = norm(meann - aircraft_pos(i,1:2));

    % plot of estimation process of particles step by step
    p = plot(PF.particles(:,2),PF.particles(:,1),'y.', ...
             aircraft_pos(i,2),aircraft_pos(i,1),'r+', ...
             meann(2),meann(1),'*b');
    p(1).MarkerSize = 1;
    p(2).MarkerSize = 5;
    p(3).MarkerSize = 5;
    legend('Particles','True Position','PF Estimation')
    xlim([aircraft_pos(i,2)-3000 aircraft_pos(i,2)+3000]);
    ylim([aircraft_pos(i,1)-3000 aircraft_pos(i,1)+3000]);
    xlabel('East(m)')
    ylabel('North(m)')
    title('One Step Particles Aircraft Motion and Particles')
    grid on
    % adding text that shows error
    text(aircraft_pos(i,2)-2500,aircraft_pos(i,1)+2500,['Distance Error = ' num2str(estim_error)],'Color','red','FontSize',10)

    % pause simulation for seeing clearly step by step
    %pause(0.1);
    clf(fig)

    % PARTICLE FILTER ALGORITHM

    %  move aircraft for taking real radar value
    %aircraft_pos = PF.aircraft_step(aircraft_pos,u);
    %radar_data = interp2(dted{1},dted{2},dted{3},aircraft_pos(1),aircraft_pos(2));
    %radar_data = alt - radar_data + randn(1)*alt_std; % adding altimeter sensor error

    %radar_data = 

    % move particles for Particle Filter algorithm and finding elevation
    % with DTED

    if i ~= step
        % move particles for Particle Filter algorithm and finding elevation
        % with DTED
        PF.particle_step(u);
        % updating weights of particles
        PF.update_weights(radar_data(:,:,i),dted)
    
        % taking mean and var variable for estimation metric
        [meann, var] = PF.estimate();
    
        % change input for seeing different case
        u(1) = u(1) + deg2rad(head_inc);
        %elev_particles_pc_history(1+N*(i-1):N*i,:) = PF.elev_particles_pc(:,:)
    end
end

%% Plotting Sim Results
%close(fig)

% 3D Figure of DTED map,particles and estimation figure
h1.visualizeDTED(ref_lla,boundary_right_upper_lla);
hold on 

nsample = round(length(aircraft_pos(:,1))/10);

part_indis = 1:N;
for i=1:9
    part_indis = [part_indis  N*nsample*i+1:N*nsample*i+1+N];
end
%part_indis = [1:N, N*nsample+1:N*nsample:length(particles_history(:,1))];

plot_particles_history = particles_history(part_indis,:);
plot_aircraft_pos = aircraft_pos(1:nsample:end,:);
plot_estimated_pos = estimated_pos(1:nsample:end,:);



particles_lla = ned2lla([plot_particles_history(:,1) plot_particles_history(:,2) -alt*ones(length(plot_particles_history(:,1)),1)],[ref_lla 0],'flat');
real_pos_lla = ned2lla([plot_aircraft_pos(:,1) plot_aircraft_pos(:,2) -alt*ones(length(plot_aircraft_pos(:,1)),1)],[ref_lla 0],'flat');
estimated_pos_lla = ned2lla([plot_estimated_pos(:,1) plot_estimated_pos(:,2) -alt*ones(length(plot_estimated_pos(:,1)),1)],[ref_lla 0],'flat');

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
p = plot(plot_particles_history(:,2),plot_particles_history(:,1),'y.', ...
                           plot_aircraft_pos(:,2),plot_aircraft_pos(:,1),'r+', ...
                           plot_estimated_pos(:,2),plot_estimated_pos(:,1),'*b');
p(1).MarkerSize = 1;
p(2).MarkerSize = 5;
p(3).MarkerSize = 5;
legend('Particles','True Position','PF Estimation')
xlabel('East(m)')
ylabel('North(m)')
title('Particles in 2D Map','FontSize',20)
grid on
axis equal

diff = aircraft_pos(:,1:2) - estimated_pos;
mean_error = mean(sqrt(diff(:,1).^2 + diff(:,1).^2));
disp(['Average error is ',num2str(mean_error),' meters'])

toc
