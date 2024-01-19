classdef StateEstimator < handle
    %StateEstimator Manages the state estimator that is based on terrain
    %matching.
    %
    %   The class allows two methods to perform location state estimation:
    %       - Particle Filter
    %       - Particle Swarm Optimzation
    %
    %   In both methods, the likelihood of the point cloud measurment can
    %   be evaluated based on one of two approachs:
    %       - Raycast from hypothetical agent x-y positions to get z
    %       - From hypothetical agent x-y positions, use the measruement
    %         point cloud x-y pattern to read z.
    %

    properties
        priorPose   % Best prior guess of the pose in world frame
                    %   positionLiDAR     x, y, z        (ENU world frame)
                    %   orientationLiDAR  psi-theta-phi  (Yaw angle of sensor frame w.r.t. to world frame)

        hReferenceMapScanner    % Scanner to be used by the estimator

        % Particle Swam Settings
        MaxIterations
        SwarmSize
        PatchXSpan
        PatchYSpan
    end

    methods (Access = public)
        function obj = StateEstimator(varargin)
            %StateEstimator Construct an instance of this class.
            %
            %   Initializes the estimator
            obj.MaxIterations   = [];
            obj.SwarmSize       = [];
            obj.PatchXSpan      = 500;
            obj.PatchYSpan      = 500;

            if nargin < 1
                % Default argument
                % Dimensions and scanner parameters
                obj.priorPose = {};
            else
                obj.priorPose = varargin{1};
            end
        end

        function param_estimate = getEstimate(obj, aPrior, ptCloudRadar, flagRAYCAST)
            %getEstimate Returns the location estimate in world frame.
            %   
            %   Takes as input the prior, the measurement point cloud, and
            %   a method flag.

            obj.priorPose = aPrior;
            obj.hReferenceMapScanner.positionLiDAR    = obj.priorPose(1:3,1);
            obj.hReferenceMapScanner.orientationLiDAR = obj.priorPose(4:6,1);

            % Search among hypothetical poses
            nvars = 2;
            xbar = obj.hReferenceMapScanner.positionLiDAR(1);
            ybar = obj.hReferenceMapScanner.positionLiDAR(2);

            lb = [xbar - obj.PatchXSpan; ybar - obj.PatchYSpan];
            ub = [xbar + obj.PatchXSpan; ybar + obj.PatchYSpan];

            fun = @(x) obj.scancorrelation(x, ptCloudRadar, flagRAYCAST);
            options = optimoptions('particleswarm');
            if ~isempty(obj.MaxIterations)
                options.MaxIterations = bj.MaxIterations;
            end
            if ~isempty(obj.SwarmSize)
                options.SwarmSize = bj.SwarmSize;
            end
            tic;
            param_estimate = particleswarm(fun,nvars,lb,ub,options);
            toc;
            disp (['Estimated [x,y]: ' '[ ' num2str(param_estimate) ' ]']);
        end
    end

    methods (Access = private)
        function corr = scancorrelation(obj, x, ptCloudRadar, flagRAYCAST)
            % This assumes that the matching needs to happen by varying x and y
            % positions only. As for altitude (z), and theta and phi angles, it is
            % assumed that on-board sensors already have this information.

            Zr = ptCloudRadar.Location(:,3);
            n = numel(Zr);

            x_particle = x(1);
            y_particle = x(2);
            obj.hReferenceMapScanner.positionLiDAR([1, 2]) = [x_particle y_particle];
            if flagRAYCAST
                obj.hReferenceMapScanner.scanTerrain(false);
                Z = obj.hReferenceMapScanner.ptCloud.Location(:,3);
            else
                XrYr = ptCloudRadar.Location(:,1:2);
                Z = zeros(n,1);
                for i=1:n
                    Z(i) = obj.hReferenceMapScanner.readAltimeter([XrYr(i,1);XrYr(i,2);0]);
                end
            end

            % TO RESOLVE LATER. Decide the best way to treat NANs.
            idx = or(isnan(Z),isnan(Zr));
            flag = 2;
            if flag == 0
                % Using world frame, with NaNs replaced by zeros
                Z = Z + obj.hReferenceMapScanner.positionLiDAR(3); Zr = Zr + obj.hReferenceMapScanner.positionLiDAR(3);
                Z(idx) = 0;
                Zr(idx) = Z(idx);
            elseif flag == 1
                % Deleting NaNs
                Z(idx) = [];
                Zr(idx) = [];
            elseif flag == 2
                % Using local frame with NaNs replaced by zero w.r.t. sensor frame
                Z(idx) = 0 - obj.hReferenceMapScanner.positionLiDAR(3);
                Zr(idx) = Z(idx);
            end

            corr = -(Zr'*Z) / sqrt((Zr'*Zr)*(Z'*Z));
        end
    end
end
