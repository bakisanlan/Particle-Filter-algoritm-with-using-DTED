classdef ParticleFilter < handle

    properties
        N
        x_range
        y_range
        hdg_range
        alt_std
        imu_std
        dt
        particles
        weights
        particles_elevation
        alt
    end
%% PF Functions
    methods
        function self = ParticleFilter(N,x_range,y_range,hdg_range,alt_std,imu_std,dt,alt)
            self.N = N;
            self.x_range = x_range;
            self.y_range = y_range;
            self.hdg_range = hdg_range;
            self.alt_std = alt_std;
            self.imu_std = imu_std;
            self.dt = dt;
            self.weights = ones(self.N,1)/self.N;
            self.alt = alt;

            self.create_uniform_particles;
        end
        
        function aircraft_pos = aircraft_step(self,aircraft_pos, u)
            % Move according to control input u (heading change, velocity)
            % with noise Q (std heading change, std velocity)

            % kinematik model of UAV
            
            % Update heading
            aircraft_pos(3) = u(1) + (randn(1) * self.imu_std(1));
            aircraft_pos(3) = mod(aircraft_pos(3), 2*pi);
            
            % Move in the (noisy) commanded direction
            velocity = (u(2) * self.dt) + (randn(1) * self.imu_std(2));
            aircraft_pos(1) = aircraft_pos(1) + cos(aircraft_pos(3)) .* velocity;
            aircraft_pos(2) = aircraft_pos(2) + sin(aircraft_pos(3)) .* velocity;
        end

        function particle_step(self, u)
            % Move according to control input u (heading change, velocity)
            % with noise Q (std heading change, std velocity)
          
            % kinematik model of UAV
            
            
            % Update heading
            self.particles(:, 3) = u(1) + (randn(self.N, 1) .* self.imu_std(1));
            self.particles(:, 3) = mod(self.particles(:, 3), 2*pi);
            
            % Move in the (noisy) commanded direction
            velocity = (u(2) * self.dt) + (randn(self.N, 1) * self.imu_std(2));
            self.particles(:, 1) = self.particles(:, 1) + cos(self.particles(:, 3)) .* velocity;
            self.particles(:, 2) = self.particles(:, 2) + sin(self.particles(:, 3)) .* velocity;
        end
        
        function update_weights(self, radar_data,dted)
            % update weight based on PDF value

            % updating elevation data of particles
            self.find_elev_particles(dted);

            self.weights = self.weights .* normpdf(radar_data,self.particles_elevation,self.alt_std);
            self.weights = self.weights + 1e-300;
            self.weights = self.weights ./ sum(self.weights);

            if self.neff < self.N/2
                indexes = self.resample_Systematic;
                self.resample(indexes)
            end
        end
        
        function [mean, var] = estimate(self)
            % taking weighted mean and var of particles for estimation 
            %%returns mean and variance of the weighted particles
            pos = self.particles(:, 1:2);
            mean = sum(pos .* self.weights) / sum(self.weights);
            var  = sum((pos - mean).^2 .* self.weights) / sum(self.weights);
        end
    
    end

%% Functions that is used in Class as subfunctions
    methods

        function create_uniform_particles(self)
            % create particles uniform distrubition near to the guess of aircraft
            % position

            self.particles = zeros(self.N,3);
            self.particles(:, 1) = unifrnd(self.x_range(1), self.x_range(2), [self.N 1]);
            self.particles(:, 2) = unifrnd(self.y_range(1), self.y_range(2), [self.N 1]);
            self.particles(:, 3) = unifrnd(self.hdg_range(1), self.hdg_range(2), [self.N 1]);
            self.particles(:, 3) = mod(self.particles(:, 3),2*pi);
        end

        function find_elev_particles(self,dted)
            % find elevation of particles from DTED
    
            self.particles_elevation = interp2(dted{1},dted{2},dted{3},self.particles(:,1),self.particles(:,2));
            self.particles_elevation = self.alt - self.particles_elevation + randn(self.N,1)*self.alt_std;
        end

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
            % calculating N_eff for triggering of resampling
            n_eff = 1 ./ sum((self.weights).^2);
        end

        function resample(self,indexes)
            % resampling from indexes that is produces by sampling methods
            self.particles = self.particles(indexes,:);
            self.weights = ones(length(self.particles),1)/length(self.particles);
        end

    end
end


