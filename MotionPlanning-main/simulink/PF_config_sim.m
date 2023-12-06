addpath(genpath(cd))
%% Sensor Property
% 0 or close to 0 altitude error cause to particles gets very very lower
% probability value from pdf so resampling will not working very well
alt_std = 3;  % altimeter sensor std error value
% IMU error provide exploration of particles rather than exploit roughly 
imu_std = [5 5];  % imu sensor imu_std(1) = V_N std,  imu_std(2) = V_E std

%% Creating dted 3D array for PF algorithm interpolation data
% 'DigitalElevationModel' class that is written by Murad Abu-Khalaf is
% used
dem = DigitalElevationModel_simu;
%ndownsample = 1;
pNED = dem.getNEDCoordinates([41.0828 28.8844],[41.1945 29.0246]);
[X,Y] = ndgrid(pNED.pN(1,:),pNED.pE(:,1));
h = -pNED.pD';
dted = {X , Y, h};
%% Particles Property
N = 400; % Number of particles
range_part = 2000; % uniformly distribute particles around aircraft with that range
x_range = [aircraft_init_pos(1,1) - 0.5*range_part ; aircraft_init_pos(1,1) + 0.5*range_part];
y_range = [aircraft_init_pos(1,2) - 0.5*range_part ; aircraft_init_pos(1,2) + 0.5*range_part];
exp_rate = 0;

%% PF Algorithm Parameters
%step = length(aircraft_pos(:,1));  % 
% dt = dx / V
sim_dt = 0.01;
dt = sim_dt;

%% Initiliazing PF Class
PF = ParticleFilter_simu(N,x_range,y_range,exp_rate,alt_std,imu_std,dt);


