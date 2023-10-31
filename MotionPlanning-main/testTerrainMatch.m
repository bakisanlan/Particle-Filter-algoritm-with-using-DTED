% testTERRAINMATCHING Performs terrain matching on SRTM DTED.
%   Uses particle swarm optimization. The aim of this code is to test that
%   the main terrain matching functionality is loading and running in an
%   open-loop way. This is not an automatic test, and needs to run and be
%   verified manually.

% Set RNG for reproducability
close all; clear;clc;
rng default

% Scanning settings
orientationLiDAR = [0 20];
positionLiDAR = [1500;1500;500];
dtheta = 10;
dphi = 10;
rayRange = 1500;

%% Scenario #1
% Get a reference radar scan from a location.
h1 = DigitalElevationModel;
h1.visualizeDTED;
dted = h1.getMetricGridElevationMap([41 29],[41.040 29.040], 10);
h = RayCasting3DMesh(dted);
h.orientationLiDAR = orientationLiDAR;
h.positionLiDAR = positionLiDAR;
h.dtheta = dtheta;
h.dphi = dphi;
obj.rayRange = rayRange;
h.showMap; axis('auto');  daspect('auto'); pbaspect([1 1 1]);
h.hFigure.CurrentAxes.Children(2).FaceColor = 'flat';
colorbar;
set(h.hFigure,'WindowStyle','docked')
[~,~,Zr] = h.scanDigitalTerrain(true);
idx = isnan(Zr); Zr(idx) = 0;

% Search among hypothetical poses
nvars = 2;
lb = [1000; 1000];
ub = [2000; 2000];
fun = @(x) scancorrelation(x,h,Zr);
options = optimoptions('particleswarm');
%options = optimoptions('particleswarm','MaxIterations',20);
%options = optimoptions('particleswarm','SwarmSize',100);
tic;
param_estimate = particleswarm(fun,nvars,lb,ub,options);
toc;
disp (['Estimated [x,y]: ' '[ ' num2str(param_estimate) ' ]']);

pause;

%% Scenario #2
% Get a reference radar scan from another location.
positionLiDAR = [1500;1500;500];
dted = h1.getMetricGridElevationMap([41.06 29.03],[41.12 29.09], 10);
h = RayCasting3DMesh(dted);
h.orientationLiDAR = orientationLiDAR;
h.positionLiDAR = positionLiDAR;
h.dtheta = dtheta;
h.dphi = dphi;
obj.rayRange = rayRange;
h.showMap; axis('auto');  daspect('auto'); pbaspect([1 1 1]);
h.hFigure.CurrentAxes.Children(2).FaceColor = 'flat';
colorbar;
set(h.hFigure,'WindowStyle','docked')
[~,~,Zr] = h.scanDigitalTerrain(true);
idx = isnan(Zr); Zr(idx) = 0;


% Search among hypothetical poses
nvars = 2;
lb = [1000; 1000];
ub = [2000; 2000];
fun = @(x) scancorrelation(x,h,Zr);
options = optimoptions('particleswarm');
%options = optimoptions('particleswarm','MaxIterations',20);
%options = optimoptions('particleswarm','SwarmSize',100);
tic;
param_estimate = particleswarm(fun,nvars,lb,ub,options);
toc;
disp (['Estimated [x,y]: ' '[ ' num2str(param_estimate) ' ]']);

function corr = scancorrelation(x,h,Zr)
% This assumes that the matching needs to happen by varying x and y
% positions only. As for altitude (z), and theta and phi angles, it is
% assumed that on-board sensors already have this information.
xp = x(1);
yp = x(2);
h.positionLiDAR([1, 2]) = [xp yp];
[~,~,Z] = h.scanDigitalTerrain(false);
idx = isnan(Z); Z(idx) = 0;
corr = -(Zr'*Z) / sqrt((Zr'*Zr)*(Z'*Z));
end