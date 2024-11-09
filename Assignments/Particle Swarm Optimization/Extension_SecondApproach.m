clear all
close all
clc
tic;

%% PSO Parameters
% Initial PSO parameters (broad search)
broad_max_iterations = 2;
broad_swarm_size = 50;
broad_w = 2;
broad_c1 = 0.5;
broad_c2 = 0.5;
broad_wdamp = 1;

% Bloodhound_c1 PSO parameters (adjusted search)
bloodhound_c1_max_iterations = 10;
bloodhound_c1_swarm_size = 10;
bloodhound_c1_w = 0.4;
bloodhound_c1_c1 = 2.5;
bloodhound_c1_c2 = 2.5;
bloodhound_wdamp = 0.9;

% Decision variables bounds
VarMin = [0, -45, 0];
VarMax = [45, 0, 2];

[broad_results, globalBest] = runPSO(broad_max_iterations, broad_swarm_size, broad_w, broad_c1, broad_c2, broad_wdamp, VarMin, VarMax, false);
duration_broad=toc;
GlobalBestParams = globalBest.Position;

% Adjust bounds around the transformed best solution
range_factor = 0.1; % 10% range around the best solution
adjusted_VarMin = max(VarMin, GlobalBestParams - range_factor * (VarMax - VarMin));
adjusted_VarMax = min(VarMax, GlobalBestParams + range_factor * (VarMax - VarMin));
tic;
[bloodhound_results, globalBest] = runPSO(bloodhound_c1_max_iterations, bloodhound_c1_swarm_size, bloodhound_c1_w, bloodhound_c1_c1, bloodhound_c1_c2, bloodhound_wdamp, adjusted_VarMin, adjusted_VarMax, false); % With velocity limit
duration_bloodhound=toc;
writetable(broad_results, 'broad_2.csv');
writetable(bloodhound_results, 'bloodhound_2.csv');
duration = duration_broad + duration_bloodhound;

disp('Best Solution:');
disp(globalBest.Position);
disp(['Best Cost: ' num2str(globalBest.Cost)]);
disp(['Duration: ' duration]);


function [broad_results, globalBest] = runPSO(max_iterations, swarm_size, w, c1, c2, wdamp,VarMin, VarMax, MaxVelOn)
    nVar = numel(VarMin);       % Number of Decision Variables
    VarSize = [1 nVar];         % Decision Variables Matrix Size

    % Initialize PSO parameters
    wdamp = wdamp;                  % Inertia Weight Damping Ratio
    MaxVel = 0.3*(VarMax-VarMin); % Max velocity
    MinVel = -MaxVel;           % Min velocity

    % Initialize particles
    particle.Position = [];
    particle.Velocity = [];
    particle.Cost = [];
    particle.Best.Position = [];
    particle.Best.Cost = [];

    globalBest.Cost = -inf;
    globalBest.Position = [];

    % Create Population Array
    pop = repmat(particle, swarm_size, 1);

    for i = 1:swarm_size
        % Initialize Position
        pop(i).Position = unifrnd(VarMin, VarMax, VarSize);

        % Initialize Velocity
        pop(i).Velocity = zeros(VarSize);

        % Evaluation
        [pop(i).Cost, ~] = simFox(pop(i).Position);

        % Update Personal Best
        pop(i).Best.Position = pop(i).Position;
        pop(i).Best.Cost = pop(i).Cost;

        % Update Global Best
        if pop(i).Best.Cost > globalBest.Cost
            globalBest = pop(i).Best;
        end
    end

    % Display initial best cost
    disp(['Initial Best Cost: ' num2str(globalBest.Cost)]);

    % Matrix to save particles' parameters, objective function values, and iteration number
    broad_results = table();

    %% PSO Main Loop
    for it = 1:max_iterations
        for i = 1:swarm_size
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
            [pop(i).Cost, ~] = simFox(pop(i).Position);

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

        % Display best cost at each iteration
        disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(globalBest.Cost)]);

        % Store iteration results
        iteration_results = array2table([repmat(it, swarm_size, 1), ...
                                         reshape([pop.Position], [], nVar), ...
                                         [pop.Cost]'], ...
                                        'VariableNames', {'Iteration', 'Param1', 'Param2', 'Param3', 'Cost'});
        broad_results = [broad_results; iteration_results];

        % Inertia Weight Damping
        w = w * wdamp;
    end
end

