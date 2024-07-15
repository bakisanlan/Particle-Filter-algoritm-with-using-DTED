classdef AircraftBot < handle
    %AircraftBot Emulates an Aircraft flying over a given terrain 
    %   This is an aircraft object to simulate the ability of an aircraft
    %   to localize based on terrain information.
    %   
    %   The model supports adding noise reflecting disturbance



    properties(SetAccess = public)
        dt              % Time step
        Pose            % Pose of the aircraft [x, y, z, psi]
        EstimatedPose   % Optimal pose estimate
        mapHandle       % DEM map handle
        WithNoise       % Enable noise
        Sigma           % Sigma^2 is variance, a 2 by 1 vector
        dx
    end

    methods
        function obj = AircraftBot(initialPose, Ts)
            %AircraftBot Construct an instance of this class
            % 
            %   Initialization of properties
            obj.Pose = reshape(initialPose, numel(initialPose),1);
            obj.dt   = Ts;
            obj.WithNoise = false;
            obj.Sigma = [5; 0.02];
            %obj.Sigma = [0; 0];

        end

        function move(obj, u, modelF)
            %MOVE Moves the aircraft based on the provided input.
            %
            %   The kinematics are represented by
            %    dx/dt      = v * cos(psi);
            %    dy/dt      = v * sin(psi);
            %    dz/dt      = 0;
            %    dpsi/dt    = omega;
            %
            %    u = [v; omega]

            % storing old state
            x_old = obj.Pose;

            if obj.WithNoise
                w = obj.Sigma.*(rand(2,1)-0.5)*2;
            else
                w = zeros(2,1);
            end

            u = reshape(u, numel(u),1); % column vector
            u_noise = u + w;

            % Updated based on input
            
            % There are 2 different kinematical model. Heading is state 
            % variable in the first model, heading is input variable 
            % in the second model. 
            if modelF == 1
                f =[0;
                    0;
                    0;
                    0];

                g =[cos(obj.Pose(4)) 0;
                    sin(obj.Pose(4)) 0;
                    0                       0;
                    0                       1];
                dPose_dt = f + g * (u_noise);
                obj.Pose = obj.Pose  + dPose_dt *  obj.dt;
            else
                dPose_dt  = [u_noise(1)*cos(u_noise(2)) ; u_noise(1)*sin(u_noise(2)) ; 0];
                obj.Pose(1:3) = obj.Pose(1:3) + dPose_dt * obj.dt;
                obj.Pose(4) = u_noise(2);
            end

            % storing delta x for TERCOM grid movement
            obj.dx = obj.Pose - x_old;


        end
    end
end