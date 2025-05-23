%% CRAI
rosshutdown;
setenv('ROS_MASTER_URI', 'http://172.22.45.57:11311');
setenv('ROS_IP','172.22.44.203'); 
rosinit();
%% MAIN
% OBJETIVOS DE EJEMPLO
x_objetivo = 5;
y_objetivo = 5;

% === Parámetros del mapa y navegación ===
resolution = 0.06;
mapSize = 15 / resolution;
logOddsMap = zeros(mapSize, mapSize);
visitedMap = false(mapSize, mapSize);  % Mapa de celdas visitadas

l_occ = log(9);
l_min = -5;
l_max = 5;
l_free = log(1/9);

odomSub = rossubscriber('/robot0/odom');
laserSub = rossubscriber('/robot0/laser_1');

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg = rosmessage(pub);

distancia_umbral = 0.5; 
tiempo_limite = 120;      % EN SEGUNDOS
start_time = tic;

% Programa automatico de recorrido del robot
while toc(start_time) < tiempo_limite
    % Obtener datos para actualizar mapa
    odomMsg = receive(odomSub, 1);
    pose = odomMsg.Pose.Pose;
    x0 = pose.Position.X;
    y0 = pose.Position.Y;
    quat = pose.Orientation;
    yaw = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = yaw(1);

    scanMsg = receive(laserSub, 1);
    ranges = scanMsg.Ranges;
    angleMin = scanMsg.AngleMin;
    angleInc = scanMsg.AngleIncrement;

    % Actualizar mapa con la función completa
    [logOddsMap, visitedMap] = actualizar_mapa_completo(x0, y0, theta, ranges, angleMin, angleInc, ...
        resolution, mapSize, logOddsMap, visitedMap, l_occ, l_free, l_min, l_max);

    % Calcular distancia actual mínima usando tu función
    distancia_actual = min_distancia_lidar();

    if distancia_actual > distancia_umbral
        avanzar();
    else
        girar_180();
        tiempo_limite2 = 1.5;      % EN SEGUNDOS
        start_time2 = tic;
        while toc(start_time2) < tiempo_limite2
            avanzar();
            % Se podría actualizar el mapa aquí si quieres
            [logOddsMap, visitedMap] = actualizar_mapa_completo(x0, y0, theta, ranges, angleMin, angleInc, ...
                resolution, mapSize, logOddsMap, visitedMap, l_occ, l_free, l_min, l_max);
        end
        girar();
    end

    % Visualizar mapa en cada iteración (opcional, puedes comentar para optimizar)
    visualizar_mapa(logOddsMap, visitedMap, x0, y0, resolution, mapSize, floor(toc(start_time)), ranges, angleMin, angleInc, theta);

end

disp("Fin primera fase");

% Programa con coordenadas de recorrido del robot
%avanzar_coordenadas(x_objetivo,y_objetivo);

% Mostrar y guardar mapa final con tu función completa
guardar_mapa_final(logOddsMap, visitedMap, resolution, mapSize);

%% --- Funciones (igual que antes, ya las tienes) ---

function [logOddsMap, visitedMap] = actualizar_mapa_completo(x0, y0, theta, ranges, angleMin, angleInc, ...
    resolution, mapSize, logOddsMap, visitedMap, l_occ, l_free, l_min, l_max)

    for r = 1:length(ranges)
        range = ranges(r);
        if range < 0.15 || range > 8.0
            continue;
        end

        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        x1 = x0 + range * cos(worldTheta);
        y1 = y0 + range * sin(worldTheta);

        nSteps = ceil(range / resolution);
        for s = 1:nSteps-1
            xStep = x0 + s * resolution * cos(worldTheta);
            yStep = y0 + s * resolution * sin(worldTheta);
            [ixStep, iyStep] = world2map(xStep, yStep, resolution, mapSize);
            if ixStep > 0 && ixStep <= mapSize && iyStep > 0 && iyStep <= mapSize
                logOddsMap(iyStep, ixStep) = max(l_min, logOddsMap(iyStep, ixStep) + l_free);
                visitedMap(iyStep, ixStep) = true;
            end
        end

        [ix1, iy1] = world2map(x1, y1, resolution, mapSize);
        if ix1 > 0 && ix1 <= mapSize && iy1 > 0 && iy1 <= mapSize
            logOddsMap(iy1, ix1) = min(l_max, logOddsMap(iy1, ix1) + l_occ);
            visitedMap(iy1, ix1) = true;
        end
    end
end

function [ix, iy] = world2map(x, y, resolution, mapHeight)
    ix = round(x / resolution) + 1;
    iy = mapHeight - round(y / resolution);
end

function visualizar_mapa(logOddsMap, visitedMap ,x0, y0, resolution, mapSize, i, ranges, angleMin, angleInc, theta)
    probMap = 1 - (1 ./ (1 + exp(logOddsMap)));  % Convertir log-odds a probabilidad
    imagesc(flipud(probMap));
    colormap(flipud(gray));
    colorbar;
    set(gca, 'YDir', 'normal');
    axis equal tight;
    hold on;
    plot((x0/resolution)+1, (y0/resolution)+1, 'bo', 'MarkerSize', 5, 'LineWidth', 2);

    % Dibujar rayos LIDAR
    for r = 1:length(ranges)
        range = ranges(r);
        if range < 0.15 || range > 8.0
            continue;
        end
        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        x_end = x0 + range * cos(worldTheta);
        y_end = y0 + range * sin(worldTheta);

        % Convertir a coordenadas de mapa
     %   [ix_start, iy_start] = world2map(x0, y0, resolution, mapSize);
      %  [ix_end, iy_end] = world2map(x_end, y_end, resolution, mapSize);

        % Graficar el rayo
        plot([x_end x0]/resolution +1, [y_end y0]/resolution +1, 'r-');
    end

    title(sprintf('Mapa con Rayos LIDAR - Iteración %d', i));
    drawnow;
    hold off;
end


function guardar_mapa_final(logOddsMap, visitedMap, resolution, mapSize)
    probMap = 1 - (1 ./ (1 + exp(logOddsMap)));
    finalMap = 0.5 * ones(mapSize, mapSize);
    finalMap(visitedMap & probMap < 0.3) = 0;
    finalMap(visitedMap & probMap > 0.7) = 1;
    figure;
    imagesc(finalMap);
    colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
    caxis([0 1]); 
    axis equal tight;
    title('Mapa final (0: libre, 0.5: desconocido, 1: ocupado)');
    colorbar('Ticks',[0 0.5 1], 'TickLabels', {'Libre','Desconocido','Ocupado'});

    binaryMap = logOddsMap > 0;
    mapObj = binaryOccupancyMap(binaryMap, 1/resolution);
    figure;
    show(mapObj);
    title('Mapa final basado en colisiones de láser');
    save('mapa_final_laser.mat', 'mapObj');

    [ixGrid, iyGrid] = find(~visitedMap);
    xCoords = (ixGrid - 1) * resolution;
    yCoords = (mapSize - iyGrid) * resolution;
    unvisitedWorldCoords = [xCoords, yCoords];
    save('celdas_no_visitadas.mat', 'unvisitedWorldCoords');

    figure;
    scatter(xCoords, yCoords, 'k.');
    title('Celdas no visitadas en coordenadas del mundo');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal tight;
end

%% FUNCION avanzar a coordenadas

function avanzar_coordenadas(x_objetivo,y_objetivo)
    distancia_umbral = 0.5;
    tolerancia = 0.25;
    destino_alcanzado = false;
    velocidad_max = 0.5;         % Velocidad máxima (m/s)
    velocidad_min = 0.05;        % Velocidad mínima al acercarse
    while ~destino_alcanzado
        [x_actual, y_actual, ~] = obtener_posicion();
        distancia_al_destino = sqrt((x_objetivo-x_actual)^2 + (y_objetivo-y_actual)^2);
        
        if distancia_al_destino < tolerancia
            detener;
            destino_alcanzado = true;
            break;
        end
        velocidad = min(velocidad_max, max(velocidad_min, 0.5*distancia_al_destino));
        distancia_actual = min_distancia_lidar();
        
        if distancia_actual > distancia_umbral
            girar_coordenadas(x_objetivo, y_objetivo);
            pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
            msg = rosmessage(pub);
            msg.Linear.X = velocidad;
            msg.Angular.Z = 0;
            send(pub, msg);
        else   
            girar_180;
            avanzar;
        end
    end
    detener;
end
%% FUNCION girar a coordenadas

function girar_coordenadas(x_objetivo,y_objetivo)
    tolerancia = 0.05;
    Kp = 0.5;
    [x_actual, y_actual, z_actual] = obtener_posicion();
    dx = x_objetivo - x_actual;
    dy = y_objetivo - y_actual;

    z_objetivo = atan2(dy,dx);
    error = inf;
    while abs(error) > tolerancia
        % Calcular error normalizado [-π, π]
        error = atan2(sin(z_objetivo - z_actual), cos(z_objetivo - z_actual));
        
        % Control proporcional
        vel_angular = Kp * error;
        
        pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
        msg = rosmessage(pub);
        msg.Angular.Z = vel_angular;
        send(pub, msg);
        [~, ~, z_actual] = obtener_posicion();
    end
    msg.Angular.Z = 0;
    send(pub, msg);
end
%% FUNCION girar_180

function girar_180()
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);
    msg.Angular.Z = 0.5;
    send(pub, msg);
    tiempo_giro = pi/abs(0.5);
    pause(tiempo_giro);
    msg.Angular.Z = 0.0;
    send(pub, msg);
end

%% FUNCION girar

function girar()
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);
    msg.Linear.X = 0.0;
    if rand() > 0.5
        msg.Angular.Z = 0.5;  % Giro antihorario
    else
        msg.Angular.Z = -0.5; % Giro horario
    end
    angulo_minimo = 0;
    angulo_maximo = 90;
    angulo_giro_deg = angulo_minimo + (angulo_maximo-angulo_minimo)*rand();
    angulo_giro = deg2rad(angulo_giro_deg);
    tiempo_giro = angulo_giro / 0.5;
    send(pub, msg);
    pause(tiempo_giro);  
    msg.Angular.Z = 0.0;
    send(pub, msg);
end

%% FUNCION obtener_yaw  

function yaw = obtener_yaw(msg_odom)  
    q = msg_odom.Pose.Pose.Orientation;  
    yaw = atan2(2.0 * (q.W * q.Z + q.X * q.Y), 1.0 - 2.0 * (q.Y^2 + q.Z^2));  
end

%% FUNCION min_distancia_lidar

function distancia_actual = min_distancia_lidar()
    laser_sub = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');
    scan_data = receive(laser_sub);
    lidar_values = scan_data.Ranges;
    distancia_actual = lidar_values(200);
    for i = 50:350
        if lidar_values(i) < distancia_actual
            distancia_actual = lidar_values(i);
            id_sensor=i;
        end
    end
end

%% FUNCION distancia_lidar

function distancia_actual = distancia_lidar(medida_lidar)
    laser_sub = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');
    scan_data = receive(laser_sub);
    lidar_values = scan_data.Ranges;
    distancia_actual = lidar_values(medida_lidar);
end

%% FUNCION avanzar

function avanzar()
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);
    msg.Linear.X = 0.3;
    msg.Angular.Z = 0.0;
    send(pub, msg);
end

%% FUNCION detener

function detener
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    msg = rosmessage(pub);
    msg.Linear.X = 0.0;
    msg.Angular.Z = 0.0;
    send(pub, msg);
end
%% FUNCIÓN obtener_posicion

function [x, y, z] = obtener_posicion()
    odom = rossubscriber('/robot0/odom');
    pause(1);
    msg_odom = odom.LatestMessage;  
    x = msg_odom.Pose.Pose.Position.X;
    y = msg_odom.Pose.Pose.Position.Y;
    z = obtener_yaw(msg_odom);    
end

