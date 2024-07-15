% This version is faster all other PF class, because there are no for loop

classdef StateEstimatorPF < handle

    properties

        priorPose   % Best prior guess of the pose in world frame
        %   positionLiDAR     x, y, z        (ENU world frame)
        %   orientationLiDAR  psi-theta-phi  (Yaw angle of sensor frame w.r.t. to world frame)

        hReferenceMapScanner    % Scanner to be used by the estimator

        % Particle Filter Variables
        N
        x_span
        y_span
        alt_std
        process_std
        particles
        exp_rate
        dt
        weights
        meann
        var
        radar_Z
        particles_pc
        MAE_particles
        MAE_particles_hist
        batch_size
        batch_n_Part
        MAE_Batch_Part
        old_index
        count_est
    end
    %% PF Functions
    methods (Access = public)
        function self = StateEstimatorPF(N,init_pos,x_span,y_span,exp_rate,alt_std,dt,batch_size)
            % Defining properties
            rng(5,'twister')

            self.N = N;
            self.x_span = x_span;
            self.y_span = y_span;
            self.alt_std = alt_std;
            self.exp_rate = exp_rate;
            self.dt = dt;
            self.batch_size = batch_size;
            self.batch_n_Part = zeros(self.N,1);
            self.process_std = [5 0.02];
            %self.process_std = [0 0];
            self.old_index = 1:self.N;
            self.count_est = 1;

            % Create uniformly distributed particles around aircraft with
            % the span range
            self.particles = self.create_particles(self.N,init_pos,[self.x_span ; self.y_span]);
            self.weights = ones(self.N,1)/self.N;

        end

        function param_estimate = getEstimate(self,prior,u,ptCloudRadar,flagRAYCAST, modelF)
            self.priorPose = prior;
            self.particle_step(u,modelF);

            %Finding MAE(mean absolute error) correlation of each 
            %particle w.r.t. radar Z value 
            self.find_MAE_particles(ptCloudRadar,flagRAYCAST);  % Nx81

            % update weight based on MAE
            self.update_weights()

            % get mean and var value and store
            self.estimate();
            param_estimate = [self.meann, self.var];

            % counting estimation times
            self.count_est = self.count_est + 1;
        end

    end

    %% Functions that is used in Class as subfunctions
    methods (Access = private)

        function particles = create_particles(self,n_partc,init_pos,span_partc)
            % create particles uniform distrubition near to the guess of aircraft
            % gps lost position

            x_range_partc = [init_pos(1)-0.5*span_partc(1) init_pos(1)+0.5*span_partc(1)];
            y_range_partc = [init_pos(2)-0.5*span_partc(2) init_pos(2)+0.5*span_partc(2)];

            particles = zeros(n_partc,4);
            % %uniform distrubition
            particles(:, 1) = unifrnd(x_range_partc(1), x_range_partc(2), [n_partc 1]);
            particles(:, 2) = unifrnd(y_range_partc(1), y_range_partc(2), [n_partc 1]);

            % % %gaussian dstrubition
            % particles(:, 1) = normrnd(init_pos(1), span_partc(1)*0.25, [n_partc 1]);
            % particles(:, 2) = normrnd(init_pos(2), span_partc(1)*0.25, [n_partc 1]);

            particles(:, 3) = init_pos(3);
            particles(:, 4) = init_pos(4);
        end

        function particle_step(self, u, modelF)
            %Move the aircraft based on the provided input.
            %
            %   The kinematics are represented by
            %    dx/dt      = v * cos(psi);
            %    dy/dt      = v * sin(psi);
            %    dz/dt      = 0;
            %    dpsi/dt    = omega;
            %
            %    u = [v; omega]
            rng(5,'twister')
            u = reshape(u, numel(u),1); % column vector
            noise = (randn(self.N, 2) .* self.process_std);
            u = u' + noise; % 

            % Updated based on input

            % different model structure
            if modelF == 1
                dPose1_dt = u(:,1) .* cos(self.particles(:,4));
                dPose2_dt = u(:,1) .* sin(self.particles(:,4));
                dPose3_dt = 0;
                dPose4_dt = u(:,2);

                self.particles(:, 1) = self.particles(:, 1) + self.dt .* dPose1_dt;% + (randn(self.N, 1) * self.process_std);
                self.particles(:, 2) = self.particles(:, 2) + self.dt .* dPose2_dt;% + (randn(self.N, 1) * self.process_std);
                self.particles(:, 3) = self.particles(:, 3) + self.dt .* dPose3_dt;
                self.particles(:, 4) = self.particles(:, 4) + self.dt .* dPose4_dt;
                
            else
                dPose1_dt = u(:,1) .* cos(u(:,2));
                dPose2_dt = u(:,1) .* sin(u(:,2));
                dPose3_dt = 0;
    
                self.particles(:, 1) = self.particles(:, 1) + self.dt .* dPose1_dt;% + (randn(self.N, 1) * self.process_std);
                self.particles(:, 2) = self.particles(:, 2) + self.dt .* dPose2_dt;% + (randn(self.N, 1) * self.process_std);
                self.particles(:, 3) = self.particles(:, 3) + self.dt .* dPose3_dt;
                self.particles(:, 4) = u(:,2);
            end
        end

        function update_weights(self)
            % Update weight based on pdf value
            % 
            % Store real radar elevation data for comparing with particles point
            % cloud radar elevation data

            % Below code block is the core of Particle Filter algorithm.
            % For each particle, we are taking mean of all n point cloud
            % radar elevation pdf(radar_elev) value. This mean pdf value
            % gives us the probability that the surface scanned from the
            % particle is the same as the actual scanned surface. After
            % finding that probabilty value, we are updating our prior
            % estimate with mean probability value for finding posterior
            % estimate(Bayes theorem)
            %

            % find means of batch elements for finding final MAE
            self.MAE_particles = cellfun(@mean,self.MAE_Batch_Part);
            self.MAE_particles_hist(:,self.count_est) = self.MAE_particles;

            self.weights = self.weights .* (1/(self.alt_std*2.506628274631)) .* exp(-0.5 .* (self.MAE_particles./self.alt_std).^2);
            %self.weights = self.weights .* (1/(self.alt_std*sqrt(2*pi))) .* exp(-0.5 .* (self.MAE_particles./self.alt_std).^2);

            self.weights = self.weights + 1e-300; % preventing 0 weights value
            self.weights = self.weights ./ sum(self.weights); % normalizing weights after each update

            % Below code for resampling criteria. There are many methods to
            % choose when we should resample but we choose N_eff < N/2 criteria
            % If condition is provided, we are resampling our particles
            % with Systematic resampling methods.
            if self.neff < self.N/2
                indexes = self.resample_Systematic;
                self.resample(indexes)
            end
        end

        function find_MAE_particles(self,ptCloudRadar,flagRAYCAST)
            % updating reference map scanner via prior estimate
            % Assume that altitude and attitudes are known
            self.hReferenceMapScanner.positionLiDAR(3)  = self.priorPose(3,1);
            self.hReferenceMapScanner.orientationLiDAR = self.priorPose(4:6,1);

            for i=1:self.N

                % aircraft terrain elevation point cloud data
                Zr_s = ptCloudRadar.s.Location(:,3);
                Zr_w = ptCloudRadar.w.Location(:,3);
                n = numel(Zr_s);

                x_particle = self.particles(i,1);
                y_particle = self.particles(i,2);
                self.hReferenceMapScanner.positionLiDAR([1, 2]) = [x_particle y_particle];
                if flagRAYCAST

                    % When Radar use RadarAltimeter for scaning terrain,
                    % ReferenceMapScanner also should use RadarAltimeter
                    if length(Zr_s) ~= 1
                        self.hReferenceMapScanner.scanTerrain(false);
                    else
                        self.hReferenceMapScanner.scanAltimeter;
                    end

                    Zs = self.hReferenceMapScanner.ptCloud.s.Location(:,3);
                    Zw = self.hReferenceMapScanner.ptCloud.w.Location(:,3);

                    particle_pc_w = self.hReferenceMapScanner.ptCloud.w.Location(:,1:3);
                else
                    XrYr_s = ptCloudRadar.s.Location(:,1:2);
                    particle_pc_w = zeros(n,3);

                    Zs = zeros(n,1);
                    Zw = zeros(n,1);

                    % Raycasting(theta=90) method for finding elevation value in sliding
                    for j=1:n
                        [Zs(j), Zw(j)] = self.hReferenceMapScanner.readAltimeter([XrYr_s(j,1);XrYr_s(j,2);Zr_s(j)]);
                        particle_pc_w(j,:) = self.hReferenceMapScanner.ptCloud.w.Location(:,1:3);
                    end
                    
                    % Interpolation method for finding elevation value in sliding
                    % [Zs, Zw] = self.hReferenceMapScanner.readAltimeter_interp([XrYr_s Zr_s]);
                    % particle_pc_w = self.hReferenceMapScanner.ptCloud.w.Location(:,1:3);
                end

                % Decide the best way to treat NANs.
                idx = or(isnan(Zw),isnan(Zr_w));
                % Deleting NaNs
                Zw(idx) = [];
                Zr_w(idx) = [];
                particle_pc_w(idx,:) = [];

                % TERCOM like historical correlation if batch_size ~=1
                % If batch_size ==1, below if condition will be
                % unnecessary but it will work.
                if (self.batch_n_Part(i) ~= self.batch_size)
                    % increase batch_n of ith particle
                    self.batch_n_Part(i) = self.batch_n_Part(i) + 1;
                    if ~isempty(Zr_w)
                        self.MAE_Batch_Part{i,1}(self.batch_n_Part(i)) = mean(abs(Zw - Zr_w),1);
                    else
                        self.MAE_Batch_Part{i,1}(self.batch_n_Part(i)) = 999;
                    end
                else
                    % drop first column of batch in every iteration
                    self.MAE_Batch_Part{i,1}(1) = [];
                    if ~isempty(Zr_w)
                        % add new MAE to end of batch array
                        self.MAE_Batch_Part{i,1}(self.batch_size) =  mean(abs(Zw - Zr_w),1);
                    else
                        self.MAE_Batch_Part{i,1}(self.batch_size) = 999;
                    end
                end

                % Storing particles point cloud measurement data in world 
                % frame for visualization
                self.particles_pc{i} = particle_pc_w;

                % Dealing empyt value with replacing NaN
                if isempty(self.particles_pc{i})
                    self.particles_pc{i} = NaN(1,3);
                end

            % Storing Radar elevation value for every estimation history
            self.radar_Z{1,self.count_est} = Zr_w;

            end
        end

        function estimate(self)
            % Taking weighted mean and var of particles for estimation
            % returns mean and variance of the weighted particles
            pos = self.particles(:,:);
            self.meann = sum(pos .* self.weights,1) / sum(self.weights);
            var_vec  = sum((pos - self.meann).^2 .* self.weights,1) / sum(self.weights);
            self.var = sqrt(var_vec(1)^2 + var_vec(2)^2);
        end

        % There are many resampling methods but we choose Systematic
        % resample
        function indx = resample_Systematic(self)
            Q = cumsum(self.weights);
            indx = zeros(1);
            T = linspace(0,1-1/self.N,self.N) + rand(1)/self.N;
            T(self.N+1) = 1;
            i=1;
            j=1;

            while (i<=self.N)
                if (T(i)<Q(j))
                    indx(i)=j;
                    i=i+1;
                else
                    j=j+1;
                end
            end
        end

        function n_eff = neff(self)
            % Calculating N_eff for triggering of resampling
            n_eff = 1 ./ sum((self.weights).^2);
        end

        function resample(self,indexes)
            % Resampling from indexes that is produces by sampling methods

            % Exploration phase
            % When resampling is activated, new samples are chosed by indexed
            % array that is produced by resampling method with some
            % 1-exploration rate. And remaining particles are created
            % uniform randomly in range of last mean estimated value with exploration rate.
            if (self.exp_rate ~= 0) && ~isempty(self.meann) 
                n_random_particles = round(self.N*self.exp_rate);
                indexes = randsample(indexes, self.N - n_random_particles);
                bounds_random_particles = [2*self.x_span ; 2*self.y_span];
                rand_particles = self.create_particles(n_random_particles,self.meann,bounds_random_particles);
                self.particles = [self.particles(indexes,:) ; rand_particles];
            else
                self.particles = self.particles(indexes,:);
            end

            disp('---------------------------------------')
            disp('--------------Resampled----------------')
            disp('---------------------------------------')

            % reset batch and batch_n of resampled particle
            self.MAE_Batch_Part(:) = {[]};
            self.batch_n_Part(:) = 0;

            % Reset weights of particles
            self.weights = ones(length(self.particles),1)/length(self.particles);
        end

    end
end


