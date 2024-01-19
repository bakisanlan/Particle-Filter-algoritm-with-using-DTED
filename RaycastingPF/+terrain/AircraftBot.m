classdef AircraftBot < handle
    %AircraftBot Emulates an Aircraft flying over a given terrain 
    %   This is an aircraft object to simulate the ability of an aircraft
    %   to localize based on terrain information.


    properties(SetAccess = public)
        dt              % Time step
        Pose            % Pose of the aircraft [x, y, z, psi]
        EstimatedPose   % Optimal pose estimate
        mapHandle       % DEM map handle
    end

    methods
        function obj = AircraftBot(initialPose, Ts)
            %AircraftBot Construct an instance of this class
            % 
            %   Initialization of properties
            obj.Pose = reshape(initialPose, numel(initialPose),1);
            obj.dt   = Ts;

        end

        function move(obj, u)
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
            f =[0;
                0;
                0;
                0];

            g =[cos(obj.Pose(4))    0;
                sin(obj.Pose(4))    0;
                0                   0;
                0                   1];

            dPose_dt = f + g * u;
            obj.Pose = obj.Pose + dPose_dt * obj.dt;
        end
    end
end