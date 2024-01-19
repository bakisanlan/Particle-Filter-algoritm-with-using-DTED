% SCANMATCH Terrain matching for an artificially generated terrain by the
% ArtificialTerrain class.
%
%   The comparision methodology utlizes particle swarms, and can be
%   compared with the approach in https://doi.org/10.1109/LRA.2020.2976310
%   

% Set RNG for reproducability
rng default

% Scanning settings
orientationLiDAR = [20 0 0];
positionLiDAR = [300;500;500];
dtheta = 5;
dpsi = 5;
rayRange = 1000;

% Get a radar scan
h1 = terrain.RayCasting3D(positionLiDAR,orientationLiDAR,rayRange,dpsi,dtheta);
h1.showMap;
h1.scanTerrain(true); 
Zr = h1.ptCloud.Location(:,3);
%idx = isnan(h1.Zr); h1.Zr(idx) = 0;

% Get a scan of the digital evlevation model
h = terrain.RayCasting3DMesh(positionLiDAR,orientationLiDAR,rayRange,dpsi,dtheta);
h.DTED = h1.aTerrain.getDTED(200);
h.showMap;
h.scanTerrain(true);

% Search among hypothetical poses
nvars = 2;
lb = [0; 0];
ub = [1000; 1000];
fun = @(x) scancorrelation(x,h,Zr);
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
h.scanTerrain(false);
Z = h.ptCloud.Location(:,3);

% TO RESOLVE LATER. Decide the best way to treat NANs.
flag = 2;
if flag == 0
    % Using world frame, with NaNs replaced by zeros
    Z = Z + h.positionLiDAR(3); Zr = Zr + h.positionLiDAR(3);
    idx = isnan(Z); Z(idx) = 0;
elseif flag ==1
    % Deleting NaNs
    idx = isnan(Z);
    Z(idx) = []; Zr(idx) = [];
elseif flag == 2
    % Using local frame with NaNs replaced by zero w.r.t. sensor frame
    idx = isnan(Z); Z(idx) = 0-h.positionLiDAR(3);
end

corr = -(Zr'*Z) / sqrt((Zr'*Zr)*(Z'*Z));
end