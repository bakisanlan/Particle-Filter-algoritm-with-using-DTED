% This version is faster all other PF class, because there are no for loop

classdef StateEstimatorTERCOM_old < handle

    properties

        priorPose   % Best prior guess of the pose in world frame
        %   positionLiDAR     x, y, z        (ENU world frame)
        %   orientationLiDAR  psi-theta-phi  (Yaw angle of sensor frame w.r.t. to world frame)

        hReferenceMapScanner    % Scanner to be used by the estimator

        % Particle Filter Settings
        N
        x_span
        y_span
        alt_std
        process_std
        particles
        exp_rate
        dt
        elev_particles_pc
        weights
        meann
        var
        Zr
        radar_Z
        particles_pc
        mean_sqrd_error
        corr
        non_idx
        batch_size
        batch_n_Part
        batch_Zr
        MAE_Batch_Part
        MAE_particles
        old_index
        count_est
        MAE_particles_hist
        MAE_particles_hist_b1

        PARAMS
        FITTING
        QUANT
        simparam
    end
    %% PF Functions
    methods (Access = public)
        function self = StateEstimatorTERCOM_old(N,init_pos,x_span,y_span,exp_rate,alt_std,dt,batch_size)
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

            self.particles = self.create_uniform_particles(self.N,init_pos,[self.x_span ; self.y_span]);
            self.weights = ones(self.N,1)/self.N;

            % % Configurations of similarity
            % PARAMS.ATTRIBUTES.GEOM = true;
            % PARAMS.ATTRIBUTES.NORM = true;
            % PARAMS.ATTRIBUTES.CURV = true;
            % PARAMS.ATTRIBUTES.COLOR = false;
            % 
            % PARAMS.ESTIMATOR_TYPE = {'VAR'};
            % PARAMS.POOLING_TYPE = {'Mean'};
            % PARAMS.NEIGHBORHOOD_SIZE = 12;
            % PARAMS.CONST = eps(1);
            % PARAMS.REF = 0;
            % self.PARAMS = PARAMS;
            % 
            % FITTING.SEARCH_METHOD = 'knn';
            % knn = 12;
            % FITTING.SEARCH_SIZE = knn;
            % self.FITTING = FITTING;
            % 
            % QUANT.VOXELIZATION = false;
            % QUANT.TARGET_BIT_DEPTH = 9;
            % self.QUANT = QUANT;
        end

        function param_estimate = getEstimate(self,prior,u,ptCloudRadar,flagRAYCAST, modelF)
            self.priorPose = prior;

            self.particle_step(u,modelF);

            %Finding MAE(mean absolute error) correlation of each 
            %particle w.r.t. radar Z value 
            self.find_MAE_particles(ptCloudRadar,flagRAYCAST);  % Nx81

            % update weight based on MAE
            self.update_weights(ptCloudRadar,flagRAYCAST)

            % store mean and var value
            [meann,var] = self.estimate();
            param_estimate = [meann,var];

            % counting estimation times
            self.count_est = self.count_est + 1;


            
            % if self.batch_n_Part == self.batch_size
            % 
            %     % update weight based on MAE
            %     self.update_weights(ptCloudRadar,flagRAYCAST)
            % 
            %     % store mean and var value
            %     [meann,var] = self.estimate();
            %     param_estimate = [meann,var];
            % 
            %     % % resetting current batch
            %     % self.batch_n = 1;
            % else 
            %     % counting number of measurement in batch
            %     self.batch_n_Part = self.batch_n_Part + 1;
            % 
            %     %param_estimate = [];
            % 
            %     % update weight based on MAE
            %     self.update_weights(ptCloudRadar,flagRAYCAST)
            % 
            %     % store mean and var value
            %     [meann,var] = self.estimate();
            %     param_estimate = [meann,var];
            % end

        end

    end

    %% Functions that is used in Class as subfunctions
    methods (Access = private)

        function particles = create_uniform_particles(self,n_partc,init_pos,span_partc)
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
            rng(5,'twister')


            %Move the aircraft based on the provided input.
            %
            %   The kinematics are represented by
            %    dx/dt      = v * cos(psi);
            %    dy/dt      = v * sin(psi);
            %    dz/dt      = 0;
            %    dpsi/dt    = omega;
            %
            %    u = [v; omega]

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

            %disp(self.particles(:,1))
        end

        function update_weights(self,ptCloudRadar,flagRAYCAST)

            % Update weight based on pdf value
            % 
            % Store real radar elevation data for comparing with particles point
            % cloud radar elevation data
            %Zr = ptCloudRadar.Location(:,3);  % 81x1

            % Below code block is the core of Particle Filter algorithm.
            % For each particle, we are taking mean of all 81 point cloud
            % radar elevation pdf(radar_elev) value. This mean pdf value
            % gives us the probability that the surface scanned from the
            % particle is the same as the actual scanned surface. After
            % finding that probabilty value, we are updating our prior
            % estimate with mean probability value for finding posterior
            % estimate(Bayes theorem)
            %

            %self.mean_sqrd_error = sqrt(mean((self.elev_particles_pc - self.Zr').^2,2));
            % a = mean((self.elev_particles_pc - self.Zr').^2,2);
            % a

            %self.alt_std = min(mean_sqrt_error);
            %corr = -(self.Zr'*self.elev_particles_pc) / sqrt((self.Zr'*self.elev_particles_pc)*(self.elev_particles_pc'*self.elev_particles_pc));

            % find means of batch elements for finding final MAE
            self.MAE_particles = cellfun(@mean,self.MAE_Batch_Part);
            self.MAE_particles = self.MAE_particles;
            self.MAE_particles_hist(:,self.count_est) = self.MAE_particles;
            %temp = cell2mat(self.MAE_Batch_Part);
            self.MAE_particles_hist_b1(:,self.count_est) = cellfun(@(x) x(end), self.MAE_Batch_Part);
            
            % % drop first column of batch array after each iteration
            % if self.batch_n_Part == self.batch_size
            %     self.MAE_Batch_Part(:,1) = [];
            % end

            %self.weights = self.weights .* (1/(self.alt_std*2.506628274631)) .* exp(-0.5 .* (self.corr./0.1).^2);
            self.weights = self.weights .* (1/(self.alt_std*sqrt(2*pi))) .* exp(-0.5 .* (self.MAE_particles./self.alt_std).^2);

            self.weights = self.weights + 1e-300; % preventing 0 weights value
            self.weights = self.weights ./ sum(self.weights); % normalizing weights after each update
            % disp(self.weights)
            % disp(self.MAE_particles)


            % Below code for resampling criteria. There are many methods to
            % choose when we should resample but we choose N_eff < N/2 criteria
            % If condition is provided, we are resampling our particles
            % with Systematic resampling methods.
            if self.neff < self.N/2
            %if false
                indexes = self.resample_Systematic;
                self.resample(indexes)
            end
              

            %disp(size(Zr'))
            %disp((self.mean_sqrd_error))
            %disp(self.corr)
            %disp(self.weights)
            % fp = [self.mean_sqrd_error self.weights];
            % fp
            %fprintf('mse %.4f, weights %.4f \n',fp);

        end

        function find_MAE_particles(self,ptCloudRadar,flagRAYCAST)

            % updating reference map scanner via prior estimate
            self.hReferenceMapScanner.positionLiDAR    = self.priorPose(1:3,1);
            self.hReferenceMapScanner.orientationLiDAR = self.priorPose(4:6,1);


            % % true altitude point cloud data
            % self.Zr = ptCloudRadar.Location(:,3);
            % n = numel(self.Zr);

            for i=1:self.N

                % true altitude point cloud data
                self.Zr = ptCloudRadar.s.Location(:,3);
                Zr_w = ptCloudRadar.w.Location(:,3);
                n = numel(self.Zr);

                x_particle = self.particles(i,1);
                y_particle = self.particles(i,2);
                self.hReferenceMapScanner.positionLiDAR([1, 2]) = [x_particle y_particle];
                %disp(self.hReferenceMapScanner.positionLiDAR)
                if flagRAYCAST

                    % When Radar use RadarAltimeter for scaning terrain,
                    % ReferenceMapScanner also should use RadarAltimeter
                    if self.hReferenceMapScanner.flagScanAltimeter
                        self.hReferenceMapScanner.scanAltimeter;
                    else
                        self.hReferenceMapScanner.scanTerrain(false);
                    end
                    Zs = self.hReferenceMapScanner.ptCloud.s.Location(:,3);
                    Zw = self.hReferenceMapScanner.ptCloud.w.Location(:,3);

                    particle_pc_w = self.hReferenceMapScanner.ptCloud.w.Location(:,1:2);
                else
                    XrYr_s = ptCloudRadar.s.Location(:,1:2);
                    XrYr_w = zeros(n,2);

                    Zs = zeros(n,1);
                    Zw = zeros(n,1);

                    for j=1:n
                        [Zs(j), Zw(j)] = self.hReferenceMapScanner.readAltimeter([XrYr_s(j,1);XrYr_s(j,2);self.Zr(j)]);
                        XrYr_w(j,:) = self.hReferenceMapScanner.ptCloud.w.Location(:,1:2);
                    end
                    % 
                    % [Zs, Zw] = self.hReferenceMapScanner.readAltimeter_interp([XrYr_s self.Zr]);
                    % XrYr_w = self.hReferenceMapScanner.ptCloud.w.Location(:,1:2);
                    % %Z(j) = self.hReferenceMapScanner.readAltimeter([XrYr(j,1);XrYr(j,2);0]);
                    
                    particle_pc_w = XrYr_w;
                end

                % TO RESOLVE LATER. Decide the best way to treat NANs.
                idx = or(isnan(Zw),isnan(Zr_w));
                flag = 1;
                if flag == 0
                    % Using world frame, with NaNs replaced by zeros
                    Z = Z + self.hReferenceMapScanner.positionLiDAR(3); self.Zr = self.Zr + self.hReferenceMapScanner.positionLiDAR(3);
                    Z(idx) = 0;
                    self.Zr(idx) = Z(idx);
                elseif flag == 1
                    % Deleting NaNs
                    self.non_idx(i,:) = idx;
                    Zw(idx) = [];
                    Zr_w(idx) = [];
                    %size(particle_pc)
                    particle_pc_w(idx,:) = [];
                    %size(particle_pc)
                elseif flag == 2
                    % Using local frame with NaNs replaced by zero w.r.t. sensor frame
                    Z(idx) = 0 - self.hReferenceMapScanner.positionLiDAR(3);
                    self.Zr(idx) = Z(idx);
                end

                %self.elev_particles_pc = zeros(self.N,length(Z));

                self.elev_particles_pc{i,self.count_est} = Zw; 

                %size(self.elev_particles_pc)

                % self.elev_particles_pc{i,self.batch_n} = Z; 
                % self.batch_Zr{self.batch_n} = self.Zr;
                
                %self.mean_sqrd_error(i,1) = sqrt(mean((self.elev_particles_pc{i} - self.Zr).^2,1));
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
                self.particles_pc{i} = [particle_pc_w Zw];
                if isempty(self.particles_pc{i})
                    self.particles_pc{i} = NaN(1,3);
                end
                self.radar_Z{1,self.count_est} = Zr_w;

                % 
                % idx_err = isnan(self.mean_sqrd_error);
                % self.mean_sqrd_error(idx_err) = 999;

                %mean((self.elev_particles_pc{i} - self.Zr).^2)
                %size(self.elev_particles_pc{i} - self.Zr)
                %self.elev_particles_pc{i}
                %self.corr(i,1) = -(self.Zr'*Z) / sqrt((self.Zr'*Z)*(Z'*Z));

                % % lineer similarity corr
                % if length(Z) > 1
                %     cor_matrix = corrcoef(self.Zr,Z); 
                %     self.corr(i,1) = cor_matrix(1,2);
                % end

                % cosine similarity corr
                %self.corr(i,1) = (self.Zr'*Z) / sqrt((self.Zr'*self.Zr)*(Z'*Z)); 
               

                % penalize the particles that have low corr coef from
                % threshold for robustness
                

                % if self.corr(i,1) < 0.2
                %     %self.mean_sqrd_error(i,1) = self.mean_sqrd_error(i,1) + 20*self.alt_std;
                %     self.mean_sqrd_error(i,1) = inf;
                % end

                %Z

                %
                % % Sort geometry
                % radarPC = ptCloudRadar;
                % particlePC = pointCloud([self.particles_pc{i}]);
                % 
                % A = radarPC;
                % B = particlePC;
                % 
                % [geomA, ~] = sortrows(A.Location);
                % A = pointCloud(geomA);
                % 
                % [geomB, ~] = sortrows(B.Location);
                % B = pointCloud(geomB);
                % 
                % % Point fusion
                % A = pc_fuse_points(A);
                % B = pc_fuse_points(B);
                % 
                % % Normals and curvatures estimation
                % [normA, curvA] = pc_estimate_norm_curv_qfit(A, self.FITTING.SEARCH_METHOD, self.FITTING.SEARCH_SIZE);
                % [normB, curvB] = pc_estimate_norm_curv_qfit(B, self.FITTING.SEARCH_METHOD, self.FITTING.SEARCH_SIZE);
                % 
                % % Set custom structs with required fields
                % sA.geom = A.Location;
                % sB.geom = B.Location;
                % if self.PARAMS.ATTRIBUTES.NORM
                %     sA.norm = normA;
                %     sB.norm = normB; 
                % end
                % if self.PARAMS.ATTRIBUTES.CURV
                %     sA.curv = curvA;
                %     sB.curv = curvB;
                % end
                % 
                % % Compute structural similarity scores
                % [pssim] = pointssim(sA, sB, self.PARAMS);
                % self.simparam{i} = pssim;

            end
        end

        function [meann,var] = estimate(self)

            % Taking weighted mean and var of particles for estimation
            % returns mean and variance of the weighted particles
            pos = self.particles(:,:);
            meann = sum(pos .* self.weights,1) / sum(self.weights);
            var  = sum((pos - meann).^2 .* self.weights,1) / sum(self.weights);

            self.meann = meann;
            self.var = sqrt(var(1)^2 + var(2)^2);
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

            % unique_idx = unique(indexes);
            % n_dead_particles = self.N - length(unique_idx);
            %
            % range_part = 2000;
            % %n_dead_particles
            % x_range_rand_partc =  [self.meann(1) - 0.5*range_part ...
            %                       self.meann(1) + 0.5*range_part];
            %
            % y_range_rand_partc =  [self.meann(2) - 0.5*range_part ...
            %                       self.meann(2) + 0.5*range_part];
            %
            % range_rand_partc = [x_range_rand_partc ; y_range_rand_partc];
            % rand_partc = self.create_uniform_particles(n_dead_particles,range_rand_partc);
            %
            % self.particles = self.particles(unique_idx,:);
            % self.particles = [self.particles ; rand_partc];

            % Exploration phase
            % When resampling is activated, new samples are chosed by indexed
            % array that is produced by resampling method with some
            % 1-exploration rate. And remaining particles are created
            % uniform randomly in range of last mean value with exploration rate.
            if (self.exp_rate ~= 0) && ~isempty(self.meann) 

                n_rand_partc = round(self.N*self.exp_rate);
                indexes = randsample(indexes, self.N - n_rand_partc);

                range_rand_part = [2*self.x_span ; 2*self.y_span];

                %disp(self.weights)
                rand_partc = self.create_uniform_particles(n_rand_partc,self.meann,range_rand_part);
                self.particles = [self.particles(indexes,:) ; rand_partc];
            else
                self.particles = self.particles(indexes,:);
            end

            disp('---------------------------------------')
            disp('--------------Resampled----------------')
            disp('---------------------------------------')

            % reset batch and batch_n of resampled particle
            % changed_part_idx = self.old_index ~= indexes;
            % self.MAE_Batch_Part(changed_part_idx) = {[]};
            % self.batch_n_Part(changed_part_idx) = 0;
            % self.old_index = indexes;


            % reset batch and batch_n of resampled particle
            %changed_part_idx = self.old_index ~= indexes;
            self.MAE_Batch_Part(:) = {[]};
            self.batch_n_Part(:) = 0;
            %self.old_index = indexes;

            self.weights = ones(length(self.particles),1)/length(self.particles);
        end

    end
end


