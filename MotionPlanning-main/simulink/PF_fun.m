function [pos_est_NED] = PF_fun(u, alt, radar_data_NED)

% input vector consist of V_N and V_E, those 2 array will be used for
% particles movement through particles kinematic model
% u = [V_N  V_E]; % input vector

% pos_cur_NED = current position of aircraft in NED

% Radar Data
% Radar Data is a 3D array that size is 81x3xSimArray. Unity radar model is
% raycasting throught DTED model with 9x9 square radar beam. Radar Data
% store each 81 points NED position relative to aircraft that touch to DTED surface.
% Those Radar Data will be used for measuring particles DTED altitude when
% they spread in sampling space through interpolation from DTED model.
% radar_data_NED = 81x3

% Move particles for Particle Filter algorithm with u matrix 
PF.particle_step(u);

% Update weights of Particles based on real data value
PF.update_weights(radar_data_NED, alt, dted_grid)

% Take mean and var variable for estimation metrics
[pos_est_NED, var] = PF.estimate();

end















