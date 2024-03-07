%pcshowpair(radar_pc,ray_pc);

% radar_pc = pcdownsample(radar_pc,"gridAverage",0.05);
% ray_pc = pcdownsample(ray_pc,"gridAverage",0.05);
% slid_pc = pcdownsample(slid_pc,"gridAverage",0.05);

radarFeature = extractFPFHFeatures(radarS_pc);
rayFeature = extractFPFHFeatures(rayS_pc);
slidFeature = extractFPFHFeatures(slidS_pc);

% % radar2Feature = extractFPFHFeatures(radar2_pt);
% % ray2Feature = extractFPFHFeatures(ray2_pc);
% % slid2Feature = extractFPFHFeatures(slid2_pc);

%[matchingPairs_ray,scores_ray] = pcmatchfeatures(slidFeature,slidFeature,slid_pc,slid_pc);

indexPairs = pcmatchfeatures(slidFeature,radarFeature,slidS_pc,radarS_pc);
matchedPts1 = select(slidS_pc,indexPairs(:,1));
matchedPts2 = select(radarS_pc,indexPairs(:,2));
pcshowMatchedFeatures(slidS_pc,radarS_pc,matchedPts1,matchedPts2,"Method","montage")


% 
% mean(scores_ray)
% 
% [matchingPairs_slid,scores_slid] = pcmatchfeatures(radarFeature,slidFeature,radar_pc,slid_pc);
% length(matchingPairs_slid)
% 
% mean(scores_slid)

% [matchingPairs_ray,scores_radar] = pcmatchfeatures(ray2Feature,slid2Feature,ray2_pc,slid2_pc);
% pcshowpair(slid2_pc,ray2_pc);

%%%%%%%%%%%%%
% maxDistance = 10; % in meters
% referenceVector = [0 0 1];
% [~,~,selectIdx] = pcfitplane(radar_pc,maxDistance,referenceVector);
% radar_pc = select(radar_pc,selectIdx,'OutputSize','full');
% [~,~,selectIdx] = pcfitplane(ray_pc,maxDistance,referenceVector);
% ray_pc = select(ray_pc,selectIdx,'OutputSize','full');

minDistance = 10; % in meters
minPoints = 1;
labels1 = pcsegdist(radar_pc,minDistance,'NumClusterPoints',minPoints);
labels2 = pcsegdist(ray_pc,minDistance,'NumClusterPoints',minPoints);

[eigFeatures1,segments1] = extractEigenFeatures(radar_pc,labels1);
[eigFeatures2,segments2] = extractEigenFeatures(ray_pc,labels2);

features1 = vertcat(eigFeatures1.Feature);
features2 = vertcat(eigFeatures2.Feature);
centroids1 = vertcat(eigFeatures1.Centroid);
centroids2 = vertcat(eigFeatures2.Centroid);

indexPairs = pcmatchfeatures(features1,features2, ...
    pointCloud(centroids1),pointCloud(centroids2));

matchedSegments1 = segments1(indexPairs(:,1));
matchedSegments2 = segments2(indexPairs(:,2));
matchedFeatures1 = eigFeatures1(indexPairs(:,1));
matchedFeatures2 = eigFeatures2(indexPairs(:,2));

figure
pcshowMatchedFeatures(matchedSegments1,matchedSegments2,matchedFeatures1,matchedFeatures2)
title('Matched Segments')



