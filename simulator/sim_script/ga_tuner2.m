function tunedGains = ga_tuner2(Drone, pop_size, maxGen, mutation_rate, kMax, tol)

% Starting Parallel computing
% parpool("Processes", 8);

% Random popolation intialization
pop = rand(pop_size, 6) .* kMax;

gen = 0;
err = tol +1;
while (gen <= maxGen && err >= tol)

    gen = gen +1;
    % Fitness function computation
    fitness = zeros(pop_size, 1);
    parfor i = 1:pop_size
        fitness(i) = compute_fitness(pop(i, :), Drone);
    end

    % New popolation
    new_pop = zeros(pop_size, 6);
    for i = 1:pop_size
        % Selection
        parent1 = roulette_selection(pop, fitness);
        parent2 = roulette_selection(pop, fitness);

        % Crossover
        child = crossover(parent1, parent2);

        % Mutation
        child = mutate(child, mutation_rate);

        % Update new popolation
        new_pop(i, :) = child;
    end

    % Update popolation
    pop = new_pop;

    % Save best individual
    [~, best_index] = min(fitness);
    best_solution = pop(best_index, :);
    fprintf('Generation #: %d\t|\tBest fitness: %f\n', gen, fitness(best_index));

    % Update error
    err = fitness(best_index);

end

    % Manual tuning                         % I set                 II set
    kP = [20;        % -> kP_T                 20                     20
          0;        % -> kP_phi                0
          0;        % -> kP_theta              0
          1.55;        % -> kP_R 50            1                      1.55
          1.55;        % -> kP_P               1                      1.55
          0.1];       % -> kP_Y                0.1                  x 0.02

    kI = [10;        % -> kI_T                 10                     10
          0;        % -> kI_phi                0
          0;        % -> kI_theta              0
          0.17;        % -> kI_R 10            0.13                   0.17
          0.17;        % -> KI_P               0.13                   0.17
          0.013];        % -> KI_Y             0.013                x 0.01

    kD = [10;        % -> kD_T                 10                     10
          0;        % -> kD_phi                0
          0;        % -> kD_theta              0
          0.76;        % -> kD_R 35            0.66                   0.76
          0.76;        % -> kD_P               0.66                   0.76
          0.066];       % -> kD_Y              0.066                x 0.01

    kP(2:3) = best_solution(1:2);
    kI(2:3) = best_solution(3:4);
    kD(2:3) = best_solution(5:6);

tunedGains = gainBuilder(kP, kI, kD);
end

%% -------- FUNCTIONS ----------
% Fitness function computation
function fitness = compute_fitness(individuo, Drone)
    % Manual tuning                         % I set                 II set
    kP = [20;        % -> kP_T                 20                     20
          0;        % -> kP_phi                0
          0;        % -> kP_theta              0
          1.55;        % -> kP_R 50            1                      1.55
          1.55;        % -> kP_P               1                      1.55
          0.1];       % -> kP_Y                0.1                  x 0.02

    kI = [10;        % -> kI_T                 10                     10
          0;        % -> kI_phi                0
          0;        % -> kI_theta              0
          0.17;        % -> kI_R 10            0.13                   0.17
          0.17;        % -> KI_P               0.13                   0.17
          0.013];        % -> KI_Y             0.013                x 0.01

    kD = [10;        % -> kD_T                 10                     10
          0;        % -> kD_phi                0
          0;        % -> kD_theta              0
          0.76;        % -> kD_R 35            0.66                   0.76
          0.76;        % -> kD_P               0.66                   0.76
          0.066];       % -> kD_Y              0.066                x 0.01

    kP(2:3) = individuo(1:2);
    kI(2:3) = individuo(3:4);
    kD(2:3) = individuo(5:6);

g = gainBuilder(kP, kI, kD);

% Simulation
Drone.updateGains(g);
parfor j = 1:length(Drone.tvec)-1
    Drone.updateState();
end

% Mean Square Error computation (MSE)
err_x = Drone.err_x(~isnan(Drone.err_x) & ~isinf(Drone.err_x));
err_y = Drone.err_y(~isnan(Drone.err_y) & ~isinf(Drone.err_y));
err_z = Drone.err_z(~isnan(Drone.err_z) & ~isinf(Drone.err_z));
err_phi = Drone.err_phi(~isnan(Drone.err_phi) & ~isinf(Drone.err_phi));
err_theta = Drone.err_theta(~isnan(Drone.err_theta) & ~isinf(Drone.err_theta));
err_psi = Drone.err_psi(~isnan(Drone.err_psi) & ~isinf(Drone.err_psi));

error = [err_x; err_y; err_z; err_phi; err_theta; err_psi];
error = error.^2;
MSE = mean(error);

% % Lyapunov Function derivative
% dV = Drone.dV(~isnan(Drone.dV) & ~isinf(Drone.dV));
% 
% V_dot = max(dV, zeros(size(dV)));
% V_dot = sum(V_dot);

% Fitness function computation
% fitness = MSE + V_dot;
fitness = MSE;
end

% Select random parent from population
function selected = roulette_selection(pop, fitness)
fitness(fitness == 0) = 1e-6;  % Avoid division by zero
prob = (1./fitness) ./ (sum(1./fitness));     % Calcola la probabilità di ogni individuo
cumprob = cumsum(prob);              % Distribuzione cumulativa
r = rand();                          % Numero casuale tra 0 e 1
selected = pop(find(cumprob >= r, 1), :);  % Seleziona un individuo
end

% Child generation
function child = crossover(parent1, parent2)
crossover_point = randi([1, 6]);  % Punto di crossover casuale
child = [parent1(1:crossover_point), parent2(crossover_point+1:end)];
end

% Child mutation
function mutated_child = mutate(child, mutation_rate)
if rand() < mutation_rate
    mutation_point = randi([1, 6]);  % Seleziona una componente a caso
    child(mutation_point) = child(mutation_point) + (rand()-0.5);  % Piccola variazione
end
mutated_child = child;
end
