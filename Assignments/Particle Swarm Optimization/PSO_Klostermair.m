clear all
close all
clc

%% PSO Parameters
nVar = 3;               % Number of Decision Variables
VarSize = [1 nVar];     % Decision Variables Matrix Size
VarMin = [0, -45, 0];   % Lower Bound of Decision Variables
VarMax = [45, 0, 2];    % Upper Bounds for each Decision Variable

MaxIt = 20;             % Maximum Number of Iterations
nPop = 10;               % Reduced Population Size (Swarm Size)
w = 0.4;                % Inertia Weight
wdamp = 1;              % Inertia Weight Damping Ratio
c1 = 2.5;               % Personal Learning Coefficient
c2 = 2.5;               % Global Learning Coefficient
MaxVelOn = true;       % Turn on/off max velocity parameter
MaxVel = 0.3*(VarMax-VarMin); % Max velocity
MinVel = -MaxVel;       % Min velocity

% Matrix to save particles' parameters, v output, and iteration number
results = [];

%% Initialization
tic;
particle.Position = [];
particle.Velocity = [];
particle.Cost = [];
particle.Best.Position = [];
particle.Best.Cost = [];

globalBest.Cost = -inf;

% Create Population Array
pop = repmat(particle, nPop, 1);

for i = 1:nPop
    % Initialize Position
    pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
    
    % Initialize Velocity
    pop(i).Velocity = zeros(VarSize);
    
    % Evaluation
    [pop(i).Cost, d] = simFox(pop(i).Position);
    
    % Save parameters and v output
    results = [results; [1, pop(i).Position, pop(i).Cost, d]];
    
    % Update Personal Best
    pop(i).Best.Position = pop(i).Position;
    pop(i).Best.Cost = pop(i).Cost;
    
    % Update Global Best
    if pop(i).Best.Cost > globalBest.Cost
        globalBest = pop(i).Best;
    end
end

%% PSO Main Loop
for it = 1:MaxIt
    for i = 1:nPop
        % Update Velocity
        pop(i).Velocity = w*pop(i).Velocity ...
            + c1*rand(VarSize).*(pop(i).Best.Position - pop(i).Position) ...
            + c2*rand(VarSize).*(globalBest.Position - pop(i).Position);
        
        if MaxVelOn
            % Apply Velocity Limits
            pop(i).Velocity = max(pop(i).Velocity, MinVel);
            pop(i).Velocity = min(pop(i).Velocity, MaxVel);
        end
        
        % Update Position
        pop(i).Position = pop(i).Position + pop(i).Velocity;
        
        % Apply Position Bounds
        pop(i).Position = max(pop(i).Position, VarMin);
        pop(i).Position = min(pop(i).Position, VarMax);
        
        % Evaluation
        [pop(i).Cost, d] = simFox(pop(i).Position);
        
        % Save parameters and v output
        results = [results; [it+1, pop(i).Position, pop(i).Cost, d]];
        
        % Update Personal Best
        if pop(i).Cost > pop(i).Best.Cost
            pop(i).Best.Position = pop(i).Position;
            pop(i).Best.Cost = pop(i).Cost;
            
            % Update Global Best
            if pop(i).Best.Cost > globalBest.Cost
                globalBest = pop(i).Best;
            end
        end
    end
    
    % Inertia Weight Damping
    w = w * wdamp;
    
    % Display Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(globalBest.Cost)]);
end

duration = toc;

%% Results
disp('Best Solution:');
disp(globalBest.Position);
disp(['Best Cost: ' num2str(globalBest.Cost)]);

% Save the results matrix to a CSV file
csvwrite('pso_results2.csv', results);
