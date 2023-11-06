clc;clear;close all;

%% SIMULATION PARAMETERS

% aircraft states and inputs
alt = 1000; % constant altitude of aircraft
velocity = 200; % m/s
heading = deg2rad(0); %rad
u = [heading ; velocity]; % input vector
gps_lost_pos = [5000 ; 5000]; % x and y position when GPS lost
aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos

% sensor property
alt_sens_std_err = 1;  % altimeter sensor std error value
imu_sens_std_err = [deg2rad(1) 10];  % imu sensor u1 = heading,  u2 = velocity
% alt_sens_std_err = 2;  % altimeter sensor std error value
% imu_sens_std_err = [deg2rad(2) 5];  % imu sensor u1 = heading,  u2 = velocity

% particles property
N = 5000; % number of particles
range_part = 3000; % initial particles range among aircraft
x_range = [gps_lost_pos(1) - 0.5*range_part ; gps_lost_pos(1) + 0.5*range_part];
y_range = [gps_lost_pos(2) - 0.5*range_part ; gps_lost_pos(2) + 0.5*range_part];
hdg_range = [0 2*pi];
particles = create_uniform_particles(x_range, y_range, hdg_range, N); %creating uniform dist. particles
weights = ones(N,1)/N;

% simulation parameters
% for preventing going to the boundary of loaded DTED map so taking NaN
% values from interp2, we added some preprocess for step value
% min_time = dist_boundary_hipo / velocity;   % the time that shortest path time
% sim_time = 100;
% sim_time = min(0.5*min_time,sim_time);
dt = 5;
step = 20;

% creating historical array for plotting purposes
real_pos = zeros(step,2);
estimated_pos = zeros(step,2);
particles_history = zeros(step*N,2);


% Finding left lower and right upper corner of DTED maps for interpolation
% and plotting
for j= 1:step
    real_pos(j,:) = aircraft_pos(1:2);
    aircraft_pos = aircraft_step(aircraft_pos,u,[0 0],dt); 
end

% reset aircraft_pos values to initial values after collecting real_pos
aircraft_pos = [gps_lost_pos ; heading];  % aircraft init pos

ref_lla = [41 29];
x_max_vehic = max(real_pos(:,1));
y_max_vehic = max(real_pos(:,2));
x_min_vehic = min(real_pos(:,1));
y_min_vehic = min(real_pos(:,2));
boundary_right_upper_lla = ned2lla([x_max_vehic y_max_vehic 0], [ref_lla 0],'flat');
boundary_left_lower_lla = ned2lla(-[x_min_vehic y_min_vehic 0], [ref_lla 0],'flat');

h1 = DigitalElevationModel;
%boundary_right_upper = [41.30 29.30];
%dist_boundary_hipo = norm(lla2ned([boundary_right_upper_lla 0], [boundary_left_lower_lla 0],'ellipsoid'));
ndownsample = 4;
ef = 1;  % expanding DTED area with a factor for preventing NaN values
dted = h1.getMetricGridElevationMap(boundary_left_lower_lla-ef, boundary_right_upper_lla+ef, ndownsample);

% % simulation parameters
% % for preventing going to the boundary of loaded DTED map so taking NaN
% % values from interp2, we added some preprocess for step value
% min_time = dist_boundary_hipo / velocity;   % the time that shortest path time
% sim_time = 100;
% sim_time = min(0.5*min_time,sim_time);
% dt = 5;
% step = round(sim_time/dt);

% initilizating estimation values
[mean, var] = estimate(particles, weights);

fig = figure(1);

% Simulation of PF
for i=1:step

    % Plotting and all other things for visulization
    %real_pos(i,:) = aircraft_pos(1:2);
    estimated_pos(i,:) = mean;
    particles_history(1+N*(i-1):N*i,:) = particles(:,1:2);

    %calculating error between estimated and real pos
    estim_error = norm(mean' - aircraft_pos(1:2));

    p = plot(particles(:,2),particles(:,1),'c.',aircraft_pos(2),aircraft_pos(1),'k+', mean(2),mean(1),'*r');
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
    %pause(1);
    clf(fig)

    % PARTICLE FILTER ALGORITHM

    %  move aircraft for taking real radar value
    aircraft_pos = aircraft_step(aircraft_pos,u,[0 0],dt);
    radar_alt = interp2(dted{1},dted{2},dted{3},aircraft_pos(1),aircraft_pos(2));
    radar_alt = alt - radar_alt + randn(1)*alt_sens_std_err; % adding altimeter sensor error

    % move particles for Particle Filter algorithm and finding elevation
    % with DTED
    particles_past = particles;
    particles = particle_step(particles, u,imu_sens_std_err ,dt);
    particles_elevation = find_elev_particles(particles,dted,alt ,alt_sens_std_err, N);

    % updating weights of particles
    weights= update_weights(particles_elevation, weights, radar_alt, alt_sens_std_err);

    % resample if needed
    if neff(weights) < N/2
        %indexes = resampleMultinomial(weights);
        %indexes = resampleResidual(weights);
        %indexes = resampleStratified(weights);
        indexes = resampleSystematic(weights);

        [particles, weights] = resample_from_index(particles, indexes);
    end

    % taking mean and var variable for estimation metric
    [mean, var] = estimate(particles, weights);

    % change input for seeing different case
    u(1) = u(1) + deg2rad(0);

end

close(fig)

% % demonstration of all step in one figure and visualization of DTED map
% boundary_left_lower = [41 29];   % left corner of DTED visualization
%                   % calulating right upper corner of DTED visualization
% x_max_vehic = max(real_pos(:,1));
% y_max_vehic = max(real_pos(:,2));
% lla = ned2lla([x_max_vehic y_max_vehic alt], [lla0 alt],'ellipsoid');

%[hF2 , hF3] = h1.visualizeDTED(boundary_left_lower_lla -ef ,boundary_right_upper_lla + ef);
[hF2 , hF3] = h1.visualizeDTED(ref_lla,boundary_right_upper_lla);

figure(1) = hF2;
figure(2) = hF3;

hold on 
particles_lla = ned2lla([particles_history(:,1) particles_history(:,2) -400*ones(length(particles_history(:,1)),1)],[ref_lla 0],'flat');
p = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'r.');%,real_pos(:,2),real_pos(:,1),700*ones(length(real_pos(:,1))),'k+', estimated_pos(:,2),estimated_pos(:,1),700*ones(length(real_pos(:,1))),'*r');
%xlim([boundary_left_lower_lla(2) boundary_right_upper_lla(2)])

% adjusting aspect ratio of X and Y data equally spaced
cur_aspect = daspect;
daspect([1 1 10000]);
set(gca,'BoxStyle','full','Box','on')


% figure(3);
% p = plot(particles_history(:,2),particles_history(:,1),'c.',real_pos(:,2),real_pos(:,1),'k+', estimated_pos(:,2),estimated_pos(:,1),'*r');
% p(1).MarkerSize = 1;
% p(2).MarkerSize = 5;
% p(3).MarkerSize = 5;
% legend('Particles','True Position','PF Estimation')
% xlabel('East(m)')
% ylabel('North(m)')
% title('Particles Aircraft Motion and Particles')
% grid on
% axis equal

%% FUNCTION THAT IS USED FOR PARTICLE FILTER

% create particles uniform distrubition near to the guess of aircraft
% position
function particles = create_uniform_particles(x_range, y_range, hdg_range, N)
particles = zeros(N,3);
particles(:, 1) = unifrnd(x_range(1), x_range(2), [N 1]);
particles(:, 2) = unifrnd(y_range(1), y_range(2), [N 1]);
particles(:, 3) = unifrnd(hdg_range(1), hdg_range(2), [N 1]);
particles(:, 3) = mod(particles(:, 3),2*pi);
end

function aircraft_pos = aircraft_step(aircraft_pos, u, std, dt)
% Move according to control input u (heading change, velocity)
% with noise Q (std heading change, std velocity)

% kinematik model of UAV

% Update heading
aircraft_pos(3) = u(1) + (randn(1) * std(1));
aircraft_pos(3) = mod(aircraft_pos(3), 2*pi);

% Move in the (noisy) commanded direction
velocity = (u(2) * dt) + (randn(1) * std(2));
aircraft_pos(1) = aircraft_pos(1) + cos(aircraft_pos(3)) .* velocity;
aircraft_pos(2) = aircraft_pos(2) + sin(aircraft_pos(3)) .* velocity;
end


function particles = particle_step(particles, u, std, dt)
% Move according to control input u (heading change, velocity)
% with noise Q (std heading change, std velocity)

% kinematik model of UAV

N = length(particles);

% Update heading
particles(:, 3) = u(1) + (randn(N, 1) .* std(1));
particles(:, 3) = mod(particles(:, 3), 2*pi);

% Move in the (noisy) commanded direction
velocity = (u(2) * dt) + (randn(N, 1) * std(2));
particles(:, 1) = particles(:, 1) + cos(particles(:, 3)) .* velocity;
particles(:, 2) = particles(:, 2) + sin(particles(:, 3)) .* velocity;
end

% find elevation of particles from DTED
function particles_elevation = find_elev_particles(particles,dted,alt ,std, N)

particles_elevation = interp2(dted{1},dted{2},dted{3},particles(:,1),particles(:,2));
particles_elevation = alt - particles_elevation + randn(N,1)*std;
end

% update weight based on PDF value
function weights = update_weights(particles_elevation, weights, radar_data, std)

weights = weights .* normpdf(radar_data,particles_elevation,std);
weights = weights + 1e-300;
weights = weights ./ sum(weights);

end

% taking weighted mean and var of particles for estimation 
function [mean, var] = estimate(particles, weights)
%%returns mean and variance of the weighted particles
pos = particles(:, 1:2);
mean = sum(pos .* weights) / sum(weights);
var  = sum((pos - mean).^2 .* weights) / sum(weights);
end

% resampling from indexes that is produces by sampling methods
function [particles, weights] = resample_from_index(particles, indexes)
particles = particles(indexes,:);
weights = ones(length(particles),1)/length(particles);
end

% calculating N_eff for triggering of resampling
function n_eff = neff(weights)
n_eff = 1 ./ sum((weights).^2);
end
