% creating trajectory from waypoints
clc;clear
load('data/topgun_newtraj.mat')
cpts = [transpose(topgun_pos(:,1)); transpose(topgun_pos(:,2))];
tpts = [0 200];
tvec = 0:2:200;
[pos, vel, ~, ~] = bsplinepolytraj(cpts,tpts,tvec);

topgun_traj_velocity = (vel(1,:).^2 + vel(2,:).^2).^0.5;
topgun_traj_velocity = topgun_traj_velocity';

tt = linspace(0,1,118);

topgun_pos = topgun_pos';
%%

topgun_traj_heading= headingFromXY(pos');
