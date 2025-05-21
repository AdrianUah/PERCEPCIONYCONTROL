%%
rosshutdown;
setenv('ROS_MASTER_URI', 'http://192.168.1.254:11311');
setenv('ROS_IP','192.168.1.201'); 
rosinit();

%% MAIN
x_objetivo = 5;
y_objetivo = 5;

odom = rossubscriber('/robot0/odom');
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg = rosmessage(pub);
laser_sub = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');
scan_data = receive(laser_sub);

distancia_umbral = 0.3; 
tiempo_limite = 120;      % EN SEGUNDOS
start_time = tic;

%programa automatico de recorrido del robot

while toc(start_time) < tiempo_limite
    distancia_actual = min_distancia_lidar;
    if distancia_actual > distancia_umbral
        avanzar;
    else
        girar_180;
        avanzar;
        pause(1);
        girar;
    end
end
disp("fin primera fase");

%programa con coordenadas de recorrido del robot
avanzar_coordenadas(x_objetivo,y_objetivo);

%% FUNCION avanzar a coordenadas

function avanzar_coordenadas(x_objetivo,y_objetivo)
    distancia_umbral = 0.3;
    tolerancia = 0.25;
    destino_alcanzado = false;
    while ~destino_alcanzado
        [x_actual, y_actual, ~] = obtener_posicion();
        distancia_al_destino = sqrt((x_objetivo-x_actual)^2 + (y_objetivo-y_actual)^2);
        
        if distancia_al_destino < tolerancia
            destino_alcanzado = true;
            break;
        end
    
        distancia_actual = min_distancia_lidar();
        
        if distancia_actual > distancia_umbral
            girar_coordenadas(x_objetivo, y_objetivo);
            avanzar;
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
    msg.Angular.Z = 0.5;
    angulo_minimo = 0;
    angulo_maximo = 359;
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