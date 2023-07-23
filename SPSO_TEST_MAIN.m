
%
% Find a path that maximizes the probability of finding object
% 

clc;
clear;
close all;

%% Problem Definition

model = CreateModel(); % Create search map and parameters

CostFunction=@(x) MyCost(x,model);    % Cost Function

nVar=model.n;       % Number of Decision Variables = searching dimension of PSO = number of path nodes

VarSize=[1 nVar];   % Size of Decision Variables Matrix

% Lower and upper Bounds of particles (Variables)
VarMin.x=model.xmin;           
VarMax.x=model.xmax;           
VarMin.y=model.ymin;           
VarMax.y=model.ymax;           
VarMin.z=model.zmin;           
VarMax.z=model.zmax;                 

VarMax.r=2*norm(model.start-model.end)/nVar;           
VarMin.r=0;

% Inclination (elevation)
AngleRange = pi/4; % Limit the angle range for better solutions
VarMin.psi=-AngleRange;            
VarMax.psi=AngleRange;          


% Azimuth 
% Determine the angle of vector connecting the start and end points
dirVector = model.end - model.start;
phi0 = atan2(dirVector(2),dirVector(1));
VarMin.phi=phi0 - AngleRange;           
VarMax.phi=phi0 + AngleRange;           


% Lower and upper Bounds of velocity
alpha=0.5;
VelMax.r=alpha*(VarMax.r-VarMin.r);    
VelMin.r=-VelMax.r;                    
VelMax.psi=alpha*(VarMax.psi-VarMin.psi);    
VelMin.psi=-VelMax.psi;                    
VelMax.phi=alpha*(VarMax.phi-VarMin.phi);    
VelMin.phi=-VelMax.phi;                    

%% PSO Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=500;           % Population Size (Swarm Size)

w=0.705;                % Inertia Weight
wdamp=0.97;         % Inertia Weight Damping Ratio
c1=0.749;             % Personal Learning Coefficient
c2=0.749;             % Global Learning Coefficient

%c1=0.51;             % Personal Learning Coefficient
%c2=0.51;             % Global Learning Coefficient

%% PSO Main Loop

% Number of runs
nRuns = 30;

% Array to hold the final best cost for each run
FinalBestCostArray = zeros(nRuns, 1);
disp(['Test for w= ', num2str(w), ', c1= ', num2str(c1), ', c2= ', num2str(c2)])
for run = 1:nRuns
    %disp(['Running PSO - Run ', num2str(run)]);
    % Create Empty Particle Structure
    empty_particle.Position=[];
    empty_particle.Velocity=[];
    empty_particle.Cost=[];
    empty_particle.Best.Position=[];
    empty_particle.Best.Cost=[];

    % Initialize Global Best
    GlobalBest.Cost=inf; % Minimization problem

    % Create an empty Particles Matrix, each particle is a solution (searching path)
    particle=repmat(empty_particle,nPop,1);
    
    BestCost=zeros(MaxIt,1);
    % Initialization Loop
    isInit = false;
    BestCostArray = zeros(nRuns, MaxIt);
    BestPositionArray = cell(nRuns, 1);
    while (~isInit)
            %disp("Initialising...");
       for i=1:nPop

            % Initialize Position
            particle(i).Position=CreateRandomSolution(VarSize,VarMin,VarMax);

            % Initialize Velocity
            particle(i).Velocity.r=zeros(VarSize);
            particle(i).Velocity.psi=zeros(VarSize);
            particle(i).Velocity.phi=zeros(VarSize);

            % Evaluation
            particle(i).Cost= CostFunction(SphericalToCart(particle(i).Position,model));

            % Update Personal Best
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;

            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
                isInit = true;
            end
        end
    end

    % PSO Main Loop
    for it = 1:MaxIt
        % ... (Ðo?n m? g?c, các bý?c PSO)
         % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;

    for i=1:nPop          
        % r Part
        % Update Velocity
        particle(i).Velocity.r = w*particle(i).Velocity.r ...
            + c1*rand(VarSize).*(particle(i).Best.Position.r-particle(i).Position.r) ...
            + c2*rand(VarSize).*(GlobalBest.Position.r-particle(i).Position.r);

        % Update Velocity Bounds
        particle(i).Velocity.r = max(particle(i).Velocity.r,VelMin.r);
        particle(i).Velocity.r = min(particle(i).Velocity.r,VelMax.r);

        % Update Position
        particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle(i).Position.r<VarMin.r | particle(i).Position.r>VarMax.r);
        particle(i).Velocity.r(OutOfTheRange)=-particle(i).Velocity.r(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.r = max(particle(i).Position.r,VarMin.r);
        particle(i).Position.r = min(particle(i).Position.r,VarMax.r);


        % psi Part

        % Update Velocity
        particle(i).Velocity.psi = w*particle(i).Velocity.psi ...
            + c1*rand(VarSize).*(particle(i).Best.Position.psi-particle(i).Position.psi) ...
            + c2*rand(VarSize).*(GlobalBest.Position.psi-particle(i).Position.psi);

        % Update Velocity Bounds
        particle(i).Velocity.psi = max(particle(i).Velocity.psi,VelMin.psi);
        particle(i).Velocity.psi = min(particle(i).Velocity.psi,VelMax.psi);

        % Update Position
        particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.psi<VarMin.psi | particle(i).Position.psi>VarMax.psi);
        particle(i).Velocity.psi(OutOfTheRange)=-particle(i).Velocity.psi(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.psi = max(particle(i).Position.psi,VarMin.psi);
        particle(i).Position.psi = min(particle(i).Position.psi,VarMax.psi);

        % Phi part
        % Update Velocity
        particle(i).Velocity.phi = w*particle(i).Velocity.phi ...
            + c1*rand(VarSize).*(particle(i).Best.Position.phi-particle(i).Position.phi) ...
            + c2*rand(VarSize).*(GlobalBest.Position.phi-particle(i).Position.phi);

        % Update Velocity Bounds
        particle(i).Velocity.phi = max(particle(i).Velocity.phi,VelMin.phi);
        particle(i).Velocity.phi = min(particle(i).Velocity.phi,VelMax.phi);

        % Update Position
        particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.phi<VarMin.phi | particle(i).Position.phi>VarMax.phi);
        particle(i).Velocity.phi(OutOfTheRange)=-particle(i).Velocity.phi(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.phi = max(particle(i).Position.phi,VarMin.phi);
        particle(i).Position.phi = min(particle(i).Position.phi,VarMax.phi);

        % Evaluation
        particle(i).Cost=CostFunction(SphericalToCart(particle(i).Position,model));

        % Update Personal Best
        if particle(i).Cost < particle(i).Best.Cost

            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;

            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end

        end
    end

        % Update Best Cost and Position for this run
        BestCostArray(run, it) = GlobalBest.Cost;
        BestPositionArray{run} = SphericalToCart(GlobalBest.Position, model);
        
    end
% Show Best Cost of the current run
    disp(['Run ', num2str(run), ': Best Cost = ', num2str(BestCostArray(run, it))]);
    % Save the final best cost for each run
    FinalBestCostArray(run) = BestCostArray(run, MaxIt);
    
end




