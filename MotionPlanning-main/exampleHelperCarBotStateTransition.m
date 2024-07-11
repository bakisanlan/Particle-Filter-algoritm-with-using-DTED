function predictParticles = exampleHelperCarBotStateTransition(pf, prevParticles, dt, u) %#ok<INUSL>
%exampleHelperCarBotStateTransition Propagate particles based on given motion model.

%   Copyright 2015-2016 The MathWorks, Inc.

    process_std = [5 0.02];
    N = length(prevParticles(:,1));

    u = reshape(u, numel(u),1); % column vector
    noise = (randn(N, 2) .* process_std);
    u = u' + noise; %
    
    % Updated based on input
    
    % different model structure
    dPose1_dt = u(:,1) .* cos(prevParticles(:,4));
    dPose2_dt = u(:,1) .* sin(prevParticles(:,4));
    dPose3_dt = 0;
    dPose4_dt = u(:,2);

    predictParticles(:, 1) = prevParticles(:, 1) + dt .* dPose1_dt;
    predictParticles(:, 2) = prevParticles(:, 2) + dt .* dPose2_dt;
    predictParticles(:, 3) = prevParticles(:, 3) + dt .* dPose3_dt;
    predictParticles(:, 4) = prevParticles(:, 4) + dt .* dPose4_dt;

end