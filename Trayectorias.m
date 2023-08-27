clear
clc

[Robot, dh, W, STLPath] = robot();

q0 = [0, 0, 0, 0, 0, 0];

% Posicion inicial
T1 = [0  0 1 5;
      0 -1 0 0;
      1  0 0 1;
      0  0 0 1];

% Posiciones intermedias mstraj

T2 = [0  0 1 3.4;
      0 -1 0 0;
      1  0 0 3.8;
      0  0 0 1];

T3 = [0 1 0 0;
      0 0 1 3.4;
      1 0 0 3.8;
      0 0 0 1];
  
% Posicion final ctraj
T4 = [0 1 0 0;
      0 0 1 5;
      1 0 0 3.2;
      0 0 0 1];

% Tiempo total del movimiento
time = 20; 

% Discretizacion
h = 200;

% Nomenclatura
% t: Trayectoria
% v: Velocidad
% a: Aceleracion
% j: Articular (interpolacion)
% c: Cartesiana (interpolacion)
% q: Coordenadas articulares
% T: Matriz de transformacion homogenea
% xyz: Coordenadas xyz

% Ejemplo
% tj_q: Trayectoria por interpolacion articular en coordenadas articulares
% tj_xyz: Trayectoria por interpolacion articular en coordenadas xyz

% Interpolacion articular

[q1, exito1] = inv_kinematics_kuka_kr_1000(Robot, dh, T1, q0, 1);

[q2, exito2] = inv_kinematics_kuka_kr_1000(Robot, dh, T2, q1, 1);

[q3, exito3] = inv_kinematics_kuka_kr_1000(Robot, dh, T3, q2, 1);

[q4, exito4] = inv_kinematics_kuka_kr_1000(Robot, dh, T4, q3, 1);

tj_q = zeros(h, 6);
tj_q(1:h/2,:) = jtraj(q1, q2, h/2);
tj_q(h/2+1:h,:) = jtraj(q2, q3, h/2);

tj_T = Robot.fkine(tj_q);

% En coordenadas cartesianas
tj_xyz = zeros(h, 3);
for i=1:h
    tj_xyz(i,:) = tj_T(i).t;
end

% Velocidad y aceleración en coordenadas articulares y cartesianas

[vj_q, aj_q] = calculate_v_a(tj_q, time/h);

[vj_xyz, aj_xyz] = calculate_v_a(tj_xyz, time/h);

% Interpolacion cartesiana
tc_T = zeros(4,4,h/2);
tc_T(:,:,1:h/2) = ctraj(T3, T4, h/2);

qant = q3;
tc_q = zeros(h/2, 6);
success = zeros(h/2, 1);  % 1 o 0 si la cinematica inversa funciona o falla
for i=1:h/2
    if i>1
        qant = tc_q(i-1,:);
    end
    [tc_q(i,:), flag] = inv_kinematics_kuka_kr_1000(Robot, dh, tc_T(:,:,i), qant, 1);
    if flag == 1
        success(i) = 1;
    end
end


tc_xyz = zeros(h/2, 3);
for i=1:h/2
    tc_xyz(i,:) = SE3(tc_T(:,:,i)).t;
end

% Velocidad y aceleración en coordenadas articulares y cartesianas

[vc_q, ac_q] = calculate_v_a(tc_q, time/(h/2));

[vc_xyz, ac_xyz] = calculate_v_a(tc_xyz, time/(h/2));


% Union de trayectorias con interpolacion articular y cartesiana

q = [tj_q(1:h,:);
     tc_q];

vq = [vj_q(1:h,:);
      vc_q];
 
aq = [aj_q(1:h,:);
      ac_q];
 
xyz = [tj_xyz(1:h,:);
       tc_xyz];
 
vxyz = [vj_xyz(1:h,:);
        vc_xyz];
 
axyz = [aj_xyz(1:h,:);
        ac_xyz];

%MSTraj

p = [q1;
     q2;
     q3;
     q4];

tacc = 1;
dt = 0.1;
speed = 0.2;

trajms = mstraj(p, speed*ones(1,6), [], q1, dt, tacc);
[m,n] = size(trajms);

trajms_se3 = Robot.fkine(trajms);

for i=1:length(trajms_se3)
   trajms_xyz(i,1:3) = trajms_se3(i).t; 
end

% Velocidad y aceleración en coordenadas articulares y cartesianas

[vtrajms_q, atrajms_q] = calculate_v_a(trajms, time/m);

[vtrajms_xyz, atrajms_xyz] = calculate_v_a(trajms_xyz, time/m);

    
% Plots
% Plot coordenadas cartesianas
time = 1:h*3/2;
figure('Name','xyz jtraj + ctraj','NumberTitle','off')
subplot(3,1,1)
plot(time, xyz(:,1), time, xyz(:,2), time, xyz(:,3))
legend('x', 'y', 'z')
title('posición')
subplot(3,1,2)
plot(time, vxyz(:,1), time, vxyz(:,2), time, vxyz(:,3))
legend('x', 'y', 'z')
title('velocidad')
subplot(3,1,3)
plot(time, axyz(:,1), time, axyz(:,2), time, axyz(:,3))
legend('x', 'y', 'z')
title('aceleracion')

time = 1:m;
figure('Name','xyz mstraj','NumberTitle','off')
subplot(3,1,1)
plot(time, trajms_xyz(:,1), time, trajms_xyz(:,2), time, trajms_xyz(:,3))
legend('x', 'y', 'z')
title('posición')
subplot(3,1,2)
plot(time, vtrajms_xyz(:,1), time, vtrajms_xyz(:,2), time, vtrajms_xyz(:,3))
legend('x', 'y', 'z')
title('velocidad')
subplot(3,1,3)
plot(time, atrajms_xyz(:,1), time, atrajms_xyz(:,2), time, atrajms_xyz(:,3))
legend('x', 'y', 'z')
title('aceleracion')

% Plot coordenadas articulares
figure('Name','q jtraj + ctraj','NumberTitle','off')
subplot(3,1,1)
title('posicion')
qplot(q)
xlabel('')
ylabel('')

subplot(3,1,2)
title('velocidad')
qplot(vq)
xlabel('')
ylabel('')

subplot(3,1,3)
title('aceleracion')
qplot(aq)
xlabel('')
ylabel('')

figure('Name','q mstraj','NumberTitle','off')
subplot(3,1,1)
title('posicion')
qplot(trajms)
xlabel('')
ylabel('')

subplot(3,1,2)
title('velocidad')
qplot(vtrajms_q)
xlabel('')
ylabel('')

subplot(3,1,3)
title('aceleracion')
qplot(atrajms_q)
xlabel('')
ylabel('')

% Trayectoria
figure('Name','Trayectoria','NumberTitle','off')
plot3(tj_xyz(:, 1), tj_xyz(:, 2), tj_xyz(:, 3))
hold on
plot3(tc_xyz(:, 1), tc_xyz(:, 2), tc_xyz(:, 3))
plot3(trajms_xyz(:, 1), trajms_xyz(:, 2), trajms_xyz(:, 3))
legend('articular','cartesiana','mstraj')
Robot.plot3d(q1, 'notiles', 'noarrow', 'path', STLPath, 'workspace', W);
%Robot.animate(q)
%Robot.animate(mstraj)

function [v, a] = calculate_v_a(q, h)  % Calculo de v y a mediante derivadas numericas
    [paso, coord] = size(q);
    v = zeros(size(q));
    a = v;
    for i=1:paso
        for j=1:coord
            if i <= 2 %O(h^2)
                v(i,j) = (-3*q(i,j) + 4*q(i+1,j) - q(i+2,j))/(2*h);
                a(i,j) = (2*q(i,j) - 5*q(i+1,j) + 4*q(i+2,j) - q(i+3,j))/h^2;
            elseif i >= paso-2  %O(h^2)
                v(i,j) = v(paso-2,j);
                a(i,j) = a(paso-2,j);   
            else  %O(h^4)
                v(i,j) = (-q(i+2,j) + 8*q(i+1,j) - 8*q(i-1,j) + q(i-2,j))/(12*h);
                a(i,j) = (-q(i+2,j) + 16*q(i+1,j) - 30*q(i,j) + 16*q(i-1,j) - q(i-2,j))/(12*h^2);
            end
        end
    end
end
      