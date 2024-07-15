%%
%%
clc;clear;
load('cor_test_GC_lowalt_init.mat')
%load('')
%%
fig = figure(1);
ray_part_dist = sqrt((TracePose(1,2)-ray_particles_pos(:,1)).^2 + ...
                (TracePose(2,2)-ray_particles_pos(:,2)).^2);

slid_part_dist = sqrt((TracePose(1,2)-slid_particles_pos(:,1)).^2 + ...
                (TracePose(2,2)-slid_particles_pos(:,2)).^2);
for i=1:200
    ray_pc = ray_part_pc_sensor{i};
    slid_pc = slid_part_pc_sensor{i};

    t = linspace(1,length(ray_pc(:,3)),length(ray_pc(:,3)));
    hold on
    plot(t,ray_pc(:,3))
    plot(t,radar_pc(:,3))
    xlim([0 126])
    ylim([-400 0])
    text(40,-300,['Corr = ' num2str(ray_cor(i))],'Color','blue','FontSize',10)
    text(40,-320,['MSE = ' num2str(ray_mse(i))],'Color','red','FontSize',10)

    pause(0.1)
    clf(fig)
end

%%
subplot(2,2,1)
plot3(ray_cor, -ray_mse,ray_part_dist,'.')
title('cor-mse-dist')
xlabel('cor')
ylabel('-mse')
zlabel('dist')

subplot(2,2,2)
plot(ray_cor, -ray_mse,'.')
title('cor-mse')
xlabel('cor')
ylabel('-mse')

subplot(2,2,3)
plot(ray_cor, ray_part_dist,'.')
title('cor-dist')
xlabel('cor')
ylabel('dist')

subplot(2,2,4)
plot(-ray_mse, ray_part_dist,'.')
title('mse-dist')
xlabel('-mse')
ylabel('dist')
%%
figure(2)
cmap = sky(5);
% Scale correlation values to colormap indices
color_indices = round(abs(slid_cor) * (size(cmap, 1) - 1)) + 1;

% Plot scatter plot with colorized points
scatter(-slid_mse, slid_part_dist, 100,cmap(color_indices, :), 'filled');
xlabel('-MSE of radar pc')
ylabel('distance to true pos')
title('Raycast-off Particles MSE-Distance based on Cor')
colormap(cmap)
colorbar


%%

figure(3)
cmap = sky(5);
% Scale correlation values to colormap indices
color_indices = round(abs(ray_cor) * (size(cmap, 1) - 1)) + 1;

% Plot scatter plot with colorized points
scatter(-ray_mse, ray_part_dist, 100,cmap(color_indices, :), 'filled');
xlabel('-MSE of radar pc')
ylabel('distance to true pos')
title('Raycast-on Particles MSE-Distance based on Cor')
colormap(cmap)
colorbar

%%
fig = figure(1);
i = 167; %50 23
 
ray_pc = ray_part_pc_sensor{i};
slid_pc = slid_part_pc_sensor{i};

t = linspace(1,length(ray_pc(:,3)),length(ray_pc(:,3)));
hold on
plot(t,radar_pc(:,3))
plot(t,ray_pc(:,3))
xlim([0 126])
ylim([-400 0])
xlabel('n th point in radar scan')
ylabel('elevation value in sensor frame')
text(40,-300,['Corr = ' num2str(ray_cor(i))],'Color','blue','FontSize',20)
text(40,-320,['MSE = ' num2str(ray_mse(i))],'Color','red','FontSize',20)
legend('Radar Pattern','Raycast-on Particle Radar Pattern','FontSize',20)

%pause(0.1)
%clf(fig)
