% This version is faster all other PF class, because there are no for loop

classdef ParticleFilter_opt < handle

    properties
        N
        x_range
        y_range
        alt_std
        imu_std
        dt
        particles
        weights
        elev_particles_pc
        alt
        meann
        var
        exp_rate
    end
%% PF Functions
    methods
        function self = ParticleFilter_opt(N,x_range,y_range,exp_rate,alt_std,imu_std,dt,alt)
            % Defining properties
            self.N = N;
            self.x_range = x_range;
            self.y_range = y_range;
            self.alt_std = alt_std;
            self.imu_std = imu_std;
            self.dt = dt;
            self.weights = ones(self.N,1)/self.N;
            self.alt = alt;
            self.exp_rate = exp_rate;

            % Create uniformly distributed particles around aircraft with the range
            self.particles = self.create_uniform_particles(self.N,[self.x_range ; self.y_range]);

        end
        

        function particle_step(self, u)

            % Move according to control input u (vn, ve)
            % with noise Q (std vn, std ve)
          
            % kinematik model of Particles
            
            % Move to particles with (noisy) commanded direction with euler
            % integration
            vn = (u(1) * self.dt) + (randn(self.N, 1) * self.imu_std(1));
            ve = (u(2) * self.dt) + (randn(self.N, 1) * self.imu_std(2));

            self.particles(:, 1) = self.particles(:, 1) + vn;
            self.particles(:, 2) = self.particles(:, 2) + ve;
            
        end
        
        function update_weights(self,radar_data,dted)

            % Update weight based on pdf value

            % Finding radar elevation datas of particles point cloud
            self.find_elev_particles_pc(radar_data,dted);  % Nx81

            % Store real radar elevation data for comparing with particles point
            % cloud radar elevation data
            radar_elev_data = radar_data(:,3);  % 81x1

            % Below code block is the core of Particle Filter algorithm.
            % For each particle, we are taking mean of all 81 point cloud
            % radar elevation pdf(radar_elev) value. This mean pdf value 
            % gives us the probability that the surface scanned from the
            % particle is the same as the actual scanned surface. After
            % finding that probabilty value, we are updating our prior
            % estimate with mean probability value for finding posterior
            % estimate(Bayes theorem)
            % 
            % for i=1:self.N
            %     mean_pdf_value_i_pc = mean(normpdf(radar_elev_data',self.elev_particles_pc(i,:),self.alt_std));
            %     self.weights(i) = self.weights(i) * mean_pdf_value_i_pc;
            % end

            mean_sqrt_error = sqrt(mean((self.elev_particles_pc - radar_elev_data').^2,2));
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
        
        function estimate(self)
    
            % Taking weighted mean and var of particles for estimation 
            % returns mean and variance of the weighted particles
            pos = self.particles(:, 1:2);
            self.meann = sum(pos .* self.weights) / sum(self.weights);
            self.var  = sum((pos - self.meann).^2 .* self.weights) / sum(self.weights);

        end

    
    end

%% Functions that is used in Class as subfunctions
    methods

        function particles = create_uniform_particles(self,n_partc,range_partc)
            % create particles uniform distrubition near to the guess of aircraft
            % gps lost position

            % first column represent North pos. of particles
            % second column represent East pos. of particles
            particles = zeros(n_partc,2);
            particles(:, 1) = unifrnd(range_partc(1,1), range_partc(1,2), [n_partc 1]);
            particles(:, 2) = unifrnd(range_partc(2,1), range_partc(2,2), [n_partc 1]);
        end

        function find_elev_particles_pc(self,radar_data,dted)

            % find elevation of particles from DTED
    
            % Each particles have information of radar point cloud
            % data(81x3)

            % First, we initialize all point cloud radar data elevation value of
            % particles in array, pc present point cloud
            
            % For each particles, we find point cloud radar position of 
            % particle. After find point cloud radar position we find
            % terrain elevation of every point from cloud through dted reference
            % map with interp2
           
            particles_pc_pos = reshape(self.particles',[1 2 self.N]) + radar_data(:,1:2);   %Array transfrom 2xN >>> 1x2xN + 81x2 >>>>>> 81x2xN
            self.elev_particles_pc = interp2(dted{1},dted{2},dted{3},particles_pc_pos(:,2,:),particles_pc_pos(:,1,:));
            self.elev_particles_pc = reshape(self.elev_particles_pc,[length(radar_data(:,1)),self.N]); % 81x1xN >>> 81xN
            self.elev_particles_pc = self.elev_particles_pc';  % 81xN >>> Nx81

            % We convert terrain elevation value to radar elevation value
            % with adding some noise
            self.elev_particles_pc = self.alt - self.elev_particles_pc + randn(self.N,length(radar_data(:,1)))*self.alt_std;    
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
    
                range_rand_part = 0.5*(-self.x_range(1) + self.x_range(2));
    
                x_range_rand_partc =  [self.meann(1) - 0.5*range_rand_part ...
                                      self.meann(1) + 0.5*range_rand_part];
    
                y_range_rand_partc =  [self.meann(2) - 0.5*range_rand_part ...
                                      self.meann(2) + 0.5*range_rand_part];
    
                range_rand_partc = [x_range_rand_partc ; y_range_rand_partc];
                rand_partc = self.create_uniform_particles(n_rand_partc,range_rand_partc);
    
                self.particles = [self.particles(sample_indx,:) ; rand_partc];
            else 
                self.particles = self.particles(indexes,:);
            end
            
            self.weights = ones(length(self.particles),1)/length(self.particles);
        end

    end
end


