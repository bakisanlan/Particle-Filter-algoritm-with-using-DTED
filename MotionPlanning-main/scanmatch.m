% SCANMATCH Performs terrain matching.

% "Iterative Closest Point" is a "Point set registration" technique useful to
% find the relative pose measurement

% "Pose graph optimization" uses many of these relative posemeasurements in
% a graph to estimate the trajectory or traversed path.

% Normal Distributions Transform (NDT) algorithm.

% The methodology used is closely related to scan matching done in
% https://doi.org/10.1109/LRA.2020.2976310


% Set RNG for reproducability
rng default

% Scanning settings
orientationLiDAR = [0 20];
positionLiDAR = [500;500;500];
dtheta = 5;
dphi = 5;

% Get a radar scan
h1 = RayCasting3D;
h1.orientationLiDAR = orientationLiDAR;
h1.positionLiDAR = positionLiDAR;
h1.showMap;
h1.dtheta = dtheta;
h1.dphi = dphi;
h1.scanRealTerrain(true);
idx = isnan(h1.Zr); h1.Zr(idx) = 0;

% Get a scan of the digital evlevation model
dted = h1.aTerrain.getDTED(200);
h = RayCasting3DMesh(dted);
h.orientationLiDAR = orientationLiDAR;
h.positionLiDAR = positionLiDAR;
h.showMap;
h.dtheta = dtheta;
h.dphi = dphi;
h.scanDigitalTerrain(true);

% Search among hypothetical poses
nvars = 2;
lb = [0; 0];
ub = [1000; 1000];
fun = @(x) scancorrelation(x,h,h1.Zr);
options = optimoptions('particleswarm');
%options = optimoptions('particleswarm','MaxIterations',20);
%options = optimoptions('particleswarm','SwarmSize',100);
tic;
param_estimate = particleswarm(fun,nvars,lb,ub,options);
toc;


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