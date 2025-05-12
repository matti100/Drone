function tunedGains = ga_tuner(Drone, pop_size, maxGen, mutation_rate, kMax, tol)

% Starting Parallel computing
% parpool("Processes", 8);

% Random popolation intialization
pop = rand(pop_size, 18) .* kMax;

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
    new_pop = zeros(pop_size, 18);
    parfor i = 1:pop_size
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

kP = best_solution(1:3:end-2);
kI = best_solution(2:3:end-1);
kD = best_solution(3:3:end);

tunedGains = gainBuilder(kP, kI, kD);
end

%% -------- FUNCTIONS ----------
% Fitness function computation
function fitness = compute_fitness(individuo, Drone)
Kp = individuo(:, 1:3:end-2);
Ki = individuo(:, 2:3:end-1);
Kd = individuo(:, 3:3:end);

g = gainBuilder(Kp, Ki, Kd);

% Simulation
Drone.updateGains(g);
parfor j = 1:length(Drone.tvec)-1
    Drone.updateState();
end

% Integral of Square Error (ISE)

err_x = Drone.Err_x;
err_y = Drone.Err_y;
err_z = Drone.Err_z;
err_phi = Drone.Err_phi;
err_theta = Drone.Err_theta;
err_psi = Drone.Err_psi;

err_x(isnan(err_x)) = eps(0);
err_y(isnan(err_y)) = eps(0);
err_z(isnan(err_z)) = eps(0);
err_phi(isnan(err_phi)) = eps(0);
err_theta(isnan(err_theta)) = eps(0);
err_psi(isnan(err_psi)) = eps(0);

err_x(isinf(err_x)) = realmax;
err_y(isinf(err_y)) = realmax;
err_z(isinf(err_z)) = realmax;
err_phi(isinf(err_phi)) = realmax;
err_theta(isinf(err_theta)) = realmax;
err_psi(isinf(err_psi)) = realmax;

error = [err_x; err_y; err_z; err_phi; err_theta; err_psi];
error = error.^2;

ISE = norm(trapz(error));

% % Lyapunov Function derivative
% dV = Drone.dV;
% dV(isnan(dV)) = eps(0);
% dV(isinf(dV)) = realmax;
%
% V_dot = max(dV, zeros(size(dV)));
% V_dot = trapz(V_dot);

% Fitness function computation
fitness = ISE;

end

% Select random parent from population
function selected = roulette_selection(pop, fitness)
fitness(fitness == 0) = eps(0);  % Avoid division by zero
prob = (1./fitness) ./ (sum(1./fitness));     % Calcola la probabilità di ogni individuo
cumprob = cumsum(prob);              % Distribuzione cumulativa
r = rand();                          % Numero casuale tra 0 e 1
selected = pop(find(cumprob >= r, 1), :);  % Seleziona un individuo
end

% Child generation
function child = crossover(parent1, parent2)
crossover_point = randi([1, 18]);  % Punto di crossover casuale
child = [parent1(1:crossover_point), parent2(crossover_point+1:end)];
end

% Child mutation
function mutated_child = mutate(child, mutation_rate)
if rand() < mutation_rate
    mutation_point = randi([1, 18]);  % Seleziona una componente a caso
    child(mutation_point) = child(mutation_point) + (rand()-0.5);  % Piccola variazione
end
mutated_child = child;
end
