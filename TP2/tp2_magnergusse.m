%% Trabalho Pratico 2
% Magner Gusse 110180
% Robótica Espacial

%% Importar ficheiros dados
clear
clc
close all


if exist('scene.mat', 'file') == 2
   load('scene.mat');
    disp('Scene loaded');
else
    warning('scene.mat does not exist');
    return;
end
% Visualizacao
visualizeScene3D(scene);


%Objetivos
if exist('tp2_config.txt', 'file') == 2
   missionObjectives = load('tp2_config.txt');
    disp('Objectives loaded');
else
    warning('tp2_config.txt does not exist');
    missionObjectives=[40 55; 120 50; -170 45; -50 60];
    
end

% Converter objetivos para cartesianos
numObjectives = size(missionObjectives, 1);
cartesianObjectives = zeros(numObjectives, 2);

for i = 1:numObjectives
    direction_deg = missionObjectives(i, 1);
    distance_m = missionObjectives(i, 2);
    direction_rad = deg2rad(direction_deg);
    cartesianObjectives(i, 1) = distance_m * cos(direction_rad);
    cartesianObjectives(i, 2) = distance_m * sin(direction_rad);
end
%% Inicializar robo e grid

currentPose=[0,0,0]; % [x, y, phi]
pathSegments = {}; 

% Limites do lidar
maxLiDAR_distance = 20;
minLiDAR_distance=0.25;
lidar_angles_deg = -180:5:180;
lidar_angles_rad = deg2rad(lidar_angles_deg);

% Mission conditions
objectiveReachedDistance = 2;
objectiveclosetowalls = 4;

% Grid and size definition
gridResolution = 0.5;
max_dist = max(vecnorm(cartesianObjectives, 2, 2)); % Distância ao objetivo mais longe
worldSize = ceil(2 * (max_dist + maxLiDAR_distance/2)); % Tamanho do mundo
worldLimits = [-worldSize/2, worldSize/2, -worldSize/2, worldSize/2];

% Vetores de coordenadas para os eixos da grelha
gridX = worldLimits(1):gridResolution:worldLimits(2);
gridY = worldLimits(3):gridResolution:worldLimits(4);
gridSize = [length(gridY), length(gridX)];

% grelha inical com 0.5,preenchida com 0 e 1
gridMap = 0.5 * ones(gridSize);

%% Visualizaçao

figure;
hold on;
axis equal;
axis(worldLimits);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
title('Simulação do Rover');

% Visualizar a grelha
h_grid = imagesc(gridX, gridY, gridMap);
set(gca, 'YDir', 'normal'); % Corrige a orientação do eixo Y

colormap(flipud(gray)); % 0->branco(Livre), 0.5->cinzento(Desconhecido), 1->preto(Ocupado).
% Não está a funcionar, funciona só cinzento e branco não sei porque

set(gca, 'CLim', [0 1]); % Fixa os limites das cores

% Visualizar o robo e o caminho
robot_vertices = [-0.5 -0.5; 0.5 -0.5; 0.5 0.5; -0.5 0.5]';
h_robot = patch(currentPose(1) + robot_vertices(1,:), currentPose(2) + robot_vertices(2,:), 'b', 'DisplayName', 'Rover');
h_path = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Trajetória ');
addpoints(h_path, currentPose(1), currentPose(2));
plot(0, 0, 'g*', 'MarkerSize', 10, 'DisplayName', 'Partida (0,0)');
plot(cartesianObjectives(:, 1), cartesianObjectives(:, 2), 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Objetivos');

%mínimos locais, inicialmente não há, depois insere-se
h_minimum = plot(NaN, NaN, 'ys', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'DisplayName', 'Mínimos Locais e obstaculos virtuais a volta');

legend('show', 'Location', 'northeastoutside');
pause(1.5)
%% Constantes

k_att = 0.5;
k_rep = 1.5;
k_rep_local_minimum = 25;

%distancia até onde o campo repulsivo afeta e distancia a percorrer por
%iteração
d_max_rep = 8;
d_max_rep_local_minimum = 15;
step_size = 0.5;


max_iterations_per_objective = 500;

stuck_steps = 7;

stuck_raio = 0.2;

stepsback = 20;


%Inicializar minimos locais
local_minimum_points = []; 



%% Inicio da missão

for i = 1:numObjectives
    current_objective = cartesianObjectives(i, :);
    h_current_objective = plot(current_objective(1), current_objective(2), 'o', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName','Alvo atual', 'Color','green');
    
    disp(['Próximo objetivo: ', num2str(i)]);
    
    current_path_segment = currentPose(1:2);
    iter = 0;
    
    %Enquanto nao chegar ao objetivo
    while norm(currentPose(1:2) - current_objective) > objectiveReachedDistance && iter < max_iterations_per_objective
        
        % deteção de mínimos locais 
        if size(current_path_segment, 1) > stuck_steps %se o tamanho for maior que 7 verificar minimo local
            if norm(current_path_segment(end,:) - current_path_segment(end-stuck_steps+1,:)) < stuck_raio % se ta no mesmo raio há 7 steps é minimo
                
                if norm(currentPose(1:2) - current_objective) <= objectiveclosetowalls %se a ddistancia for maior menor que 4 o objetivo esta proximo de paredes 
                    disp('Objetivo perto de obstáculo');
                    break; %avança para o próximo objetivo
                end
                disp('Mínimo local detetado!');
                stuck_point = currentPose(1:2);% ponto do minimo
                surrounding_points= currentPose(1:2)+[1 0; -1 0; 0 1; 0 -1; 1 1; -1 1; -1 -1; 1 -1]*2;%bloquear os pontos ao redor tambem
                local_minimum_points = [local_minimum_points; stuck_point;surrounding_points];%guardar para usar como obstaculos
                
                % Atualizacao do plot dos mínimos locais
                set(h_minimum, 'XData', local_minimum_points(:,1), 'YData', local_minimum_points(:,2));
                

                %Voltar 20 passos atras pra garantir que volta o suficiente
                for b = 1:min(stepsback, size(current_path_segment, 1) - 1)
                    
                    previous_point = current_path_segment(end-1, :);
                    %calcular a nova direçao
                    direction_vector = current_path_segment(end-2, :) - previous_point;
                    new_phi = atan2(direction_vector(2), direction_vector(1));
                    
                    % apaga essa trajetoria do path
                    currentPose = [previous_point, new_phi];
                    
                    current_path_segment = current_path_segment(1:end-1, :);
                    %atualizar o plot
                    addpoints(h_path, currentPose(1), currentPose(2));
                    R = [cos(new_phi) -sin(new_phi); sin(new_phi) cos(new_phi)];
                    rotated_vertices = R * robot_vertices;
                   
                    set(h_robot, 'XData', currentPose(1) + rotated_vertices(1,:), 'YData', currentPose(2) + rotated_vertices(2,:));
                    drawnow; pause(0.05);
                end
                
            end
        end

        % LiDAR scan para atualizar a grid
        scan_hits = [];
        for theta = lidar_angles_rad
            [distance, hitPoint] = singleBeamLiDAR(scene, currentPose, theta);
            
            % Atualizar grid
            points_along_beam = linspace(0, distance, ceil(distance / gridResolution));
            world_points_free = currentPose(1:2) + [cos(currentPose(3) + theta) * points_along_beam', sin(currentPose(3) + theta) * points_along_beam'];
            grid_indices_free = world2grid(world_points_free, gridX(1), gridY(1), gridResolution, gridSize);
            linear_indices_free = sub2ind(gridSize, grid_indices_free(:,1), grid_indices_free(:,2));

            unknown_indices = (gridMap(linear_indices_free) == 0.5);
            indices_to_update_to_free = linear_indices_free(unknown_indices);
            gridMap(indices_to_update_to_free) = 0;
            
            % Verificar se está dentro das leituras aceitaveis e só preenche se for uma posição até agora desconhecida
            if distance > minLiDAR_distance && distance < maxLiDAR_distance 
                scan_hits = [scan_hits; hitPoint];
                grid_index_obs = world2grid(hitPoint, gridX(1), gridY(1), gridResolution, gridSize);
                if gridMap(grid_index_obs(1), grid_index_obs(2)) == 0.5
                    gridMap(grid_index_obs(1), grid_index_obs(2)) = 1; 
                end
            end
        end
        

        % Cálculo de forças 
        F_atrativa = k_att * (current_objective - currentPose(1:2));
        F_repulsiva_obstaculos = [0, 0];
        if ~isempty(scan_hits)% se não tiver nada nos obstaculo
            for k = 1:size(scan_hits, 1)
                dist_vec_to_obs = currentPose(1:2) - scan_hits(k,:);
                dist_to_obs = norm(dist_vec_to_obs);
                if dist_to_obs < d_max_rep && dist_to_obs > 0%só contam distancias entre 0 e 8
                    F_rep_k = k_rep * (1/dist_to_obs - 1/d_max_rep) * (1/dist_to_obs^2) * (dist_vec_to_obs / dist_to_obs);
                    F_repulsiva_obstaculos = F_repulsiva_obstaculos + F_rep_k;
                end
            end
        end
        %Mesma coisa para os minimos
        F_repulsiva_minimos = [0, 0];
        if ~isempty(local_minimum_points)
            for k = 1:size(local_minimum_points, 1)
                dist_vec_to_min = currentPose(1:2) - local_minimum_points(k,:);
                dist_to_min = norm(dist_vec_to_min);
                if dist_to_min < d_max_rep_local_minimum && dist_to_min > 0
                    F_rep_m = k_rep_local_minimum * (1/dist_to_min - 1/d_max_rep_local_minimum) * (1/dist_to_min^2) * (dist_vec_to_min / dist_to_min);
                    F_repulsiva_minimos = F_repulsiva_minimos + F_rep_m;
                end
            end
        end

        
        F_total = F_atrativa + F_repulsiva_obstaculos + F_repulsiva_minimos;%força total
        %calculo da nova posição
        new_phi = atan2(F_total(2), F_total(1));
        velocity = [cos(new_phi), sin(new_phi)];
        new_position_xy = currentPose(1:2) + step_size * velocity;
        %atualizar nova posição
        currentPose = [new_position_xy, new_phi];
        current_path_segment = [current_path_segment; currentPose(1:2)];
        
        
        % atualizar Visualização
        set(h_grid, 'CData', gridMap); % Atualiza a imagem da grelha
        addpoints(h_path, currentPose(1), currentPose(2));%trajeto
       
        %Grafico do rover
        R = [cos(new_phi) -sin(new_phi); sin(new_phi) cos(new_phi)];
        rotated_vertices = R * robot_vertices;
        set(h_robot, 'XData', currentPose(1) + rotated_vertices(1,:), 'YData', currentPose(2) + rotated_vertices(2,:));
        drawnow;
        
        iter = iter + 1;%contador de iterações por objetivo
    end
    
    % Se ultrapassar as 500 iterações, passar ao proximo objetivo
    if iter >= max_iterations_per_objective
        warning('Limite de iterações atingido. A posição atual será considerada um mínimo local.');
        stuck_point = currentPose(1:2);
        local_minimum_points = [local_minimum_points; stuck_point];
       
        set(h_minimum, 'XData', local_minimum_points(:,1), 'YData', local_minimum_points(:,2));
    else
        disp(['Objetivo ', num2str(i), ' alcançado!']);
    end
    
    pathSegments{end+1} = current_path_segment;
    delete(h_current_objective);
end
disp('Missão concluída!');
% Preparar e guardar o ficheiro de resultados
outputMatrix = [];


for i = 1:length(pathSegments)%preencher do segmento 1 até 4(N)
    outputMatrix = [outputMatrix; pathSegments{i}];
    if i < length(pathSegments)
        outputMatrix = [outputMatrix; NaN NaN];
    end
end

%ficheiro de trajeto
writematrix(outputMatrix, 'tp2_110180.txt');
disp('Ficheiro do trajeto guardado como: tp2_110180');
%% Funções

function grid_indices = world2grid(world_pos, gridX_origin, gridY_origin, gridResolution, gridSize)


    % Calcula os índices da coluna a partir das posições x y
    col_indices = floor((world_pos(:,1) - gridX_origin) / gridResolution) + 1;
   
    row_indices = floor((world_pos(:,2) - gridY_origin) / gridResolution) + 1;
  
    col_indices = max(1, min(gridSize(2), col_indices));
    row_indices = max(1, min(gridSize(1), row_indices));
    
    grid_indices = [row_indices, col_indices];
end

% funcao do gemini e parte das forças do gemini