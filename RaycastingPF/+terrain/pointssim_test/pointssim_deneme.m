clear all;
close all;
clc;


%% Configurations
PARAMS.ATTRIBUTES.GEOM = true;
PARAMS.ATTRIBUTES.NORM = true;
PARAMS.ATTRIBUTES.CURV = true;
PARAMS.ATTRIBUTES.COLOR = false;

PARAMS.ESTIMATOR_TYPE = {'VAR'};
PARAMS.POOLING_TYPE = {'Mean'};
PARAMS.NEIGHBORHOOD_SIZE = 12;
PARAMS.CONST = eps(1);
PARAMS.REF = 0;

FITTING.SEARCH_METHOD = 'knn';
if strcmp(FITTING.SEARCH_METHOD, 'rs')
    ratio = 0.01;
elseif strcmp(FITTING.SEARCH_METHOD, 'knn')
    knn = 12;
end
FITTING.SEARCH_SIZE = [];

QUANT.VOXELIZATION = false;
QUANT.TARGET_BIT_DEPTH = 9;


%% Load point clouds
load('data/pc.mat')
A = slid_pc;
B = radar_pc;

% load('data/pc2.mat')
% A = slid2_pc;
% B = ray2_pc;
% 
% load('data/pcS.mat')
% A = slidS_pc;
% B = rayS_pc;

AA = A;
BB = B;


%% Sort geometry
[geomA, idA] = sortrows(A.Location);
if ~isempty(A.Color)
    colorA = A.Color(idA, :);
    A = pointCloud(geomA, 'Color', colorA);
else
    A = pointCloud(geomA);
end

[geomB, idB] = sortrows(B.Location);
if ~isempty(B.Color)
    colorB = B.Color(idB, :);
    B = pointCloud(geomB, 'Color', colorB);
else
    B = pointCloud(geomB);
end


%% Point fusion
A = pc_fuse_points(A);
B = pc_fuse_points(B);


%% Voxelization
if QUANT.VOXELIZATION
    A = pc_vox_scale(A, [], QUANT.TARGET_BIT_DEPTH);
    B = pc_vox_scale(B, [], QUANT.TARGET_BIT_DEPTH);
end


%% Normals and curvatures estimation
if PARAMS.ATTRIBUTES.NORM || PARAMS.ATTRIBUTES.CURV
    if strcmp(FITTING.SEARCH_METHOD, 'rs')
        FITTING.SEARCH_SIZE = round(ratio * double(max(max(A.Location) - min(A.Location))));
    else
        FITTING.SEARCH_SIZE = knn;
    end
    [normA, curvA] = pc_estimate_norm_curv_qfit(A, FITTING.SEARCH_METHOD, FITTING.SEARCH_SIZE);
    [normB, curvB] = pc_estimate_norm_curv_qfit(B, FITTING.SEARCH_METHOD, FITTING.SEARCH_SIZE);
end


%% Set custom structs with required fields
sA.geom = A.Location;
sB.geom = B.Location;
if PARAMS.ATTRIBUTES.NORM
    sA.norm = normA;
    sB.norm = normB; 
end
if PARAMS.ATTRIBUTES.CURV
    sA.curv = curvA;
    sB.curv = curvB;
end
if PARAMS.ATTRIBUTES.COLOR
    sA.color = A.Color;
    sB.color = B.Color;
end


%% Compute structural similarity scores
[pssim] = pointssim(sA, sB, PARAMS);

%pcshowpair(AA,BB)

