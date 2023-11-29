clc;clear;close all;
addpath(genpath(cd))

%% SIMULATION PARAMETERS

%% Loading Unity circular traj time series data as 'out'
% Unity out has dt = 0.01 and t_final = 250s;
load("trimmedCircularTrajData.mat") 

%% Unity out sampling parameters
% sample_space is a quantitiy refer that how often real data is taken 
% 1 value is same with unity data.
% When sample_space close to 1, algorithm will give lesser error. Because
% inputs that moves to particles are also choosen according to sample_space,
% so integration time step also reduce that cause to translation wrongly of
% particle w.r.t real aircraft motion.
sample_space = 100;      % number of sample space, value 1 is same with unity data
init_t = 3;              % because of first 2 sample is broken from unity we take samples after 2
unity_dt = 0.01;
tf = 20/unity_dt;       % we choose to take samples until 200 s.

%% Take Real aircraft States and Inputs
aircraft_pos = out.logsout.find('xyz_m').Values.Data(init_t:tf,:);  % real aircraft posisiton(Unity)
aircraft_pos(:,3) = -aircraft_pos(:,3);   % converting up z to down z
aircraft_pos = aircraft_pos(1:sample_space:end,:);
% Assumed that Aircraft and particles have constant altitude
alt = -out.logsout.find('xyz_m').Values.Data(init_t,3);     % constant altitude of aircraft

V_N = out.logsout.find('NorthDot').Values.Data(init_t:tf);  % velocity in North direction
V_N = V_N(1:sample_space:end,:);
V_E = out.logsout.find('EastDot').Values.Data(init_t:tf);   % velocity in East direction
V_E = V_E(1:sample_space:end,:);
% input vector consist of V_N and V_E, those 2 array will be used for
% particles movement through particles kinematic model
u = [V_N  V_E]; % input vector

%% Sensor Property
% 0 or close to 0 altitude error cause to particles gets very very lower
% probability value from pdf so resampling will not working very well
alt_std = 3;  % altimeter sensor std error value
% IMU error provide exploration of particles rather than exploit roughly 
imu_std = [5 5];  % imu sensor imu_std(1) = V_N std,  imu_std(2) = V_E std

%% Radar Data
% Radar Data is a 3D array that size is 81x3xSimArray. Unity radar model is
% raycasting throught DTED model with 9x9 square radar beam. Radar Data
% store each 81 points NED position relative to aircraft that touch to DTED surface.
% Those Radar Data will be used for measuring particles DTED altitude when
% they spread in sampling space through interpolation from DTED model.
radar_data = out.logsout.find('allRadarPoint_Body_m').Values.Data(:,:,init_t:tf);
radar_data = radar_data(:,:,sample_space+1:sample_space:end);

%% Finding baundary of DTED map based on aircraft motion 
ref_lla = [41.10071636898535, 29.024554581047795];  % itu arc
x_max_vehic = max(aircraft_pos(:,1));
y_max_vehic = max(aircraft_pos(:,2));
x_min_vehic = min(aircraft_pos(:,1));
y_min_vehic = min(aircraft_pos(:,2));
emapf = 2000;                 %expanding map in metrees to N and E direction for better visulization
boundary_right_upper_lla = ned2lla([x_max_vehic+emapf y_max_vehic+emapf alt], [ref_lla alt],'flat');
boundary_left_lower_lla  = ned2lla([x_min_vehic-emapf y_min_vehic-emapf alt], [ref_lla alt],'flat');

% Converting real aircraft position to LLA w.r.t ref_lla
aircraft_pos_lla = ned2lla(aircraft_pos,[ref_lla alt],"flat");

% Finding aircraft position in NED w.r.t boundary left lower corner point
% because 'h1.getMetricGridElevationMap' assume the reference point is left
% lower corner.
aircraft_pos_rel_leftlow = lla2ned([aircraft_pos_lla(:,1:2) -ones(length(aircraft_pos_lla(:,1)),1)*alt],boundary_left_lower_lla,"flat");

%% Creating dted 3D array for PF algorithm interpolation data
% 'DigitalElevationModel' class that is written by Murad Abu-Khalaf is
% used
h1 = DigitalElevationModel;
ndownsample = 1;
dted = h1.getMetricGridElevationMap(boundary_left_lower_lla, boundary_right_upper_lla, ndownsample);

%% Particles Property
N = 400; % Number of particles
range_part = 2000; % uniformly distribute particles around aircraft with that range
x_range = [aircraft_pos_rel_leftlow(1,1) - 0.5*range_part ; aircraft_pos_rel_leftlow(1,1) + 0.5*range_part];
y_range = [aircraft_pos_rel_leftlow(1,2) - 0.5*range_part ; aircraft_pos_rel_leftlow(1,2) + 0.5*range_part];

%% PF Algorithm Parameters
step = length(aircraft_pos(:,1));  % 
% dt = dx / V
dt = norm([aircraft_pos(1,1)-aircraft_pos(2,1), aircraft_pos(1,2)-aircraft_pos(2,2)])/sqrt(V_N(1)^2 + V_E(2)^2);

%% Initiliazing PF Class
PF = ParticleFilter_circ(N,x_range,y_range,alt_std,imu_std,dt,alt);

%% initilizating estimation values
[meann, var] = PF.estimate();

%% Creating historical array for plotting purposes
estimated_pos = zeros(step,2);
particles_history = zeros(step*N,2);
elev_particles_pc_history = zeros(step*N,81);

%% Figure for seeing estimation process of particles step by step 
fig = figure(1);

k = 1;  
%% Simulation of Particle Filter
tic
for i=1:step

    % For demonstration of PF process by percent
    if mod(i,round(step/100)) == 0
        fprintf('PF algorithm %%%4.1f done\n',k)
        k = min(k+1,100);
    end

    % Store all 'estimated_pos' from PF and particles history for visulization
    estimated_pos(i,:) = meann;
    particles_history(1+N*(i-1):N*i,:) = PF.particles(:,1:2);

    %calculating error between estimated and real pos
    estim_error = norm(meann - aircraft_pos_rel_leftlow(i,1:2));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If you want you see algorithm phase step by step you can uncomment below code part 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % % plot of estimation process of particles step by step
    % p = plot(PF.particles(:,2),PF.particles(:,1),'y.', ...
    %          aircraft_pos_rel_leftlow(i,2),aircraft_pos_rel_leftlow(i,1),'r+', ...
    %          meann(2),meann(1),'*b');
    % p(1).MarkerSize = 5;
    % p(2).MarkerSize = 5;
    % p(3).MarkerSize = 5;
    % legend('Particles','True Position','PF Estimation')
    % xlim([aircraft_pos_rel_leftlow(i,2)-3000 aircraft_pos_rel_leftlow(i,2)+3000]);
    % ylim([aircraft_pos_rel_leftlow(i,1)-3000 aircraft_pos_rel_leftlow(i,1)+3000]);
    % xlabel('East(m)')
    % ylabel('North(m)')
    % title('One Step Particles Aircraft Motion and Particles')
    % grid on
    % % adding text that shows error
    % text(aircraft_pos_rel_leftlow(i,2)-2500,aircraft_pos_rel_leftlow(i,1)+2500,['Distance Error = ' num2str(estim_error)],'Color','red','FontSize',10)
    % % pause simulation for seeing clearly step by step
    % pause(0.1);
    % clf(fig)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % RUN PARTICLE FILTER ALGORITHM
    %tic
    if i ~= step   % this condition is added for PF Algorithm step = length(unity data)-1
        % For understanding how algorithm works, examine the
        % 'ParticleFilter_circ' Class.
        
        % Move particles for Particle Filter algorithm with u matrix 
        PF.particle_step(u(i,:));
        
        % Update weights of Particles based on real data value
        PF.update_weights(radar_data(:,:,i),dted)
    
        % Take mean and var variable for estimation metrics
        [meann, var] = PF.estimate();
    end
    %toc
end
toc
%% Plotting Sim Results
% Below codes just for the visualization 

close(fig)

%% 3D FIGURE OF DTED MAP,PARTICLES AND ESTIMATION FIGURE
h1.visualizeDTED(boundary_left_lower_lla,boundary_right_upper_lla);
hold on 

nsample = round(length(aircraft_pos_rel_leftlow(:,1))/20);
nsample = 2;


plot_aircraft_pos = aircraft_pos_rel_leftlow(1:nsample:end,:);
plot_estimated_pos = estimated_pos(1:nsample:end,:);

part_indis = 1:N;
for i=1:length(plot_estimated_pos(:,1))-1
    part_indis = [part_indis  N*nsample*i+1:N*nsample*i+N];
end

plot_particles_history = particles_history(part_indis,:);

particles_lla = ned2lla([plot_particles_history(:,1) plot_particles_history(:,2) -alt*ones(length(plot_particles_history(:,1)),1)],boundary_left_lower_lla,'flat');
real_pos_lla = ned2lla([plot_aircraft_pos(:,1) plot_aircraft_pos(:,2) -alt*ones(length(plot_aircraft_pos(:,1)),1)],boundary_left_lower_lla,'flat');
estimated_pos_lla = ned2lla([plot_estimated_pos(:,1) plot_estimated_pos(:,2) -alt*ones(length(plot_estimated_pos(:,1)),1)],boundary_left_lower_lla,'flat');

p = plot3(particles_lla(:,2),particles_lla(:,1),particles_lla(:,3),'y.', ...
          real_pos_lla(:,2),real_pos_lla(:,1),real_pos_lla(:,3),'r+'   , ...
          estimated_pos_lla(:,2),estimated_pos_lla(:,1),estimated_pos_lla(:,3),'*b');
p(1).MarkerSize = 5;
p(2).MarkerSize = 5;
p(3).MarkerSize = 5;
legend({'DTED Mesh','Particles','True Position','PF Estimation'},Location="best")
title('Particles in 3D DTED Map','FontSize',20)
grid on
% adjusting aspect ratio of X and Y data equally spaced
cur_aspect = daspect;
daspect([1 1 10000]);
set(gca,'BoxStyle','full','Box','on')

% 2D FIGURE OF PARTICLES AND ESTIMATION FIGURE
figure(2);
p = plot(plot_particles_history(:,2),plot_particles_history(:,1),'y.', ...
                           plot_aircraft_pos(:,2),plot_aircraft_pos(:,1),'r+', ...
                           plot_estimated_pos(:,2),plot_estimated_pos(:,1),'*b');
p(1).MarkerSize = 5;
p(2).MarkerSize = 5;
p(3).MarkerSize = 5;
legend('Particles','True Position','PF Estimation')
xlabel('East(m)')
ylabel('North(m)')
title('Particles in 2D Map','FontSize',20)
grid on
axis equal

diff = aircraft_pos_rel_leftlow(:,1:2) - estimated_pos;
mean_error = mean(sqrt(diff(:,1).^2 + diff(:,2).^2));
disp(['Average error is ',num2str(mean_error),' meters'])

