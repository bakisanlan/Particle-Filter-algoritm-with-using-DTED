% This version is faster all other PF class, because there are no for loop

classdef StateEstimatorPF < handle

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
        particles
        exp_rate
        dt
        elev_particles_pc
        weights
        meann
        var
    end
    %% PF Functions
    methods (Access = public)
        function self = StateEstimatorPF(N,init_pos,x_span,y_span,exp_rate,alt_std,dt)
            % Defining properties
            self.N = N;
            self.x_span = x_span;
            self.y_span = y_span;
            self.alt_std = alt_std;
            self.exp_rate = exp_rate;
            self.dt = dt;
            % Create uniformly distributed particles around aircraft with
            % the span range

            self.particles = self.create_uniform_particles(self.N,init_pos,[self.x_span ; self.y_span]);
            self.weights = ones(self.N,1)/self.N;
        end

        function param_estimate = getEstimate(self,u,ptCloudRadar,flagRAYCAST)

            self.particle_step(u);

            self.update_weights(ptCloudRadar,flagRAYCAST)

            [meann,var] = self.estimate();

            param_estimate = [meann,var];

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
            particles(:, 1) = unifrnd(x_range_partc(1), x_range_partc(2), [n_partc 1]);
            particles(:, 2) = unifrnd(y_range_partc(1), y_range_partc(2), [n_partc 1]);
            particles(:, 3) = init_pos(3);
            particles(:, 4) = init_pos(4);
        end

        function particle_step(self, u)

            %MOVE Moves the aircraft based on the provided input.
            %
            %   The kinematics are represented by
            %    dx/dt      = v * cos(psi);
            %    dy/dt      = v * sin(psi);
            %    dz/dt      = 0;
            %    dpsi/dt    = omega;
            %
            %    u = [v; omega]

            u = reshape(u, numel(u),1); % column vector

            % Updated based on input
            dPose1_dt = u(1) .* cos(self.particles(:,4));
            dPose2_dt = u(1) .* sin(self.particles(:,4));
            dPose3_dt = 0;
            dPose4_dt = u(2);

            % vn = (u(1) * self.dt) + (randn(self.N, 1) * self.imu_std(1));
            % ve = (u(2) * self.dt) + (randn(self.N, 1) * self.imu_std(2));

            self.particles(:, 1) = self.particles(:, 1) + self.dt .* dPose1_dt;
            self.particles(:, 2) = self.particles(:, 2) + self.dt .* dPose2_dt;
            self.particles(:, 3) = self.particles(:, 3) + self.dt .* dPose3_dt;
            self.particles(:, 4) = self.particles(:, 4) + self.dt .* dPose4_dt;

        end

        function update_weights(self,ptCloudRadar,flagRAYCAST)

            % Update weight based on pdf value

            % Finding radar elevation datas of particles point cloud
            self.find_elev_particles_pc(ptCloudRadar,flagRAYCAST);  % Nx81

            % Store real radar elevation data for comparing with particles point
            % cloud radar elevation data
            Zr = ptCloudRadar.Location(:,3);  % 81x1

            % Below code block is the core of Particle Filter algorithm.
            % For each particle, we are taking mean of all 81 point cloud
            % radar elevation pdf(radar_elev) value. This mean pdf value
            % gives us the probability that the surface scanned from the
            % particle is the same as the actual scanned surface. After
            % finding that probabilty value, we are updating our prior
            % estimate with mean probability value for finding posterior
            % estimate(Bayes theorem)
            %

            mean_sqrt_error = sqrt(mean((self.elev_particles_pc - Zr').^2,2));
            self.weights = self.weights .* (1/(self.alt_std*2.506628274631)) .* exp(-0.5 .* (mean_sqrt_error./self.alt_std).^2);
            %self.weights = self.weights .* (1/(self.alt_std*sqrt(2*pi))) .* exp(-0.5 .* (mean_sqrd_error./self.alt_std).^2);

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

        function find_elev_particles_pc(self,ptCloudRadar,flagRAYCAST)

            % true altitude point cloud data
            Zr = ptCloudRadar.Location(:,3);
            n = numel(Zr);

            self.elev_particles_pc = zeros(self.N,n);

            for i=1:length(self.N)

                x_particle = self.particles(i,1);
                y_particle = self.particles(i,2);
                self.hReferenceMapScanner.positionLiDAR([1, 2]) = [x_particle y_particle];
                disp(self.hReferenceMapScanner.positionLiDAR)
                if flagRAYCAST
                    self.hReferenceMapScanner.scanTerrain(false);
                    Z = self.hReferenceMapScanner.ptCloud.Location(:,3);
                else
                    XrYr = ptCloudRadar.Location(:,1:2);
                    Z = zeros(n,1);
                    for j=1:n
                        Z(j) = self.hReferenceMapScanner.readAltimeter([XrYr(j,1);XrYr(j,2);0]);
                    end
                end

                Z
                % TO RESOLVE LATER. Decide the best way to treat NANs.
                idx = or(isnan(Z),isnan(Zr));
                flag = 2;
                if flag == 0
                    % Using world frame, with NaNs replaced by zeros
                    Z = Z + self.hReferenceMapScanner.positionLiDAR(3); Zr = Zr + self.hReferenceMapScanner.positionLiDAR(3);
                    Z(idx) = 0;
                    Zr(idx) = Z(idx);
                elseif flag == 1
                    % Deleting NaNs
                    Z(idx) = [];
                    Zr(idx) = [];
                elseif flag == 2
                    % Using local frame with NaNs replaced by zero w.r.t. sensor frame
                    Z(idx) = 0 - self.hReferenceMapScanner.positionLiDAR(3);
                    Zr(idx) = Z(idx);
                end

                self.elev_particles_pc(i,:) = Z;
                %Z
            end
        end

        function [meann,var] = estimate(self)

            % Taking weighted mean and var of particles for estimation
            % returns mean and variance of the weighted particles
            pos = self.particles(:,:);
            meann = sum(pos .* self.weights) / sum(self.weights);
            var  = sum((pos - meann).^2 .* self.weights) / sum(self.weights);

            self.meann = meann;
            self.var = var;
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
            if self.exp_rate ~= 0

                n_rand_partc = round(self.N*self.exp_rate);
                sample_indx = randsample(indexes, self.N - n_rand_partc);

                range_rand_part = [0.5*self.x_span ; 0.5*self.y_span];

                rand_partc = self.create_uniform_particles(n_rand_partc,self.meann,range_rand_part);

                self.particles = [self.particles(sample_indx,:) ; rand_partc];
            else
                self.particles = self.particles(indexes,:);
            end

            self.weights = ones(length(self.particles),1)/length(self.particles);
        end

    end
end


