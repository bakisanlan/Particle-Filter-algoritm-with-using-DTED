% a = DTED();
% 
% a.ConvertToFlat([41.10071636898535, 29.024554581047795]);
% 
% dted = a.flatMap;
% dted(:,3) = -dted(:,3); 

lla0 = [41.10071636898535, 29.024554581047795];
ru = [42 30];
lu = [42 28];
rl = [41 30];
ll = [41 28];


h1 = DigitalElevationModel;
ndownsample = 1;
dted_ru = h1.getMetricGridElevationMap(lla0, ru, ndownsample);

dted_lu = h1.getMetricGridElevationMap([lla0(1) lu(2)], [lu(1) lla0(2)], ndownsample);
dted_lu{1} = flip(dted_lu{1}) - max(dted_lu{1});

dted_rl = h1.getMetricGridElevationMap([rl(1) lla0(2)] , [lla0(1) rl(2)] , ndownsample);
dted_rl{2} = flip(dted_rl{2}) - max(dted_rl{2});

dted_ll = h1.getMetricGridElevationMap(ll,lla0, ndownsample);
dted_ll{1} = flip(dted_ll{1}) - max(dted_ll{1});
dted_ll{2} = flip(dted_ll{2}) - max(dted_ll{2});

 
