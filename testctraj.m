clear all
clc
[Robot, dh, W, STLPath] = robot();

q0 = [0, 0, 0, 0, 0, 0];

% Posicion inicial
T1 = [0  0 1 5;
      0 -1 0 0;
      1  0 0 1;
      0  0 0 1];
  
  
% Posicion intermedia

T2 = [0  0 1 3.4;
      0 -1 0 0;
      1  0 0 3.8;
      0  0 0 1];

T3 = [0  0 1 4;
      0 -1 0 4;
      1  0 0 3.8;
      0  0 0 1];
  
% Posicion final
T4 = [0 1 0 0;
      0 0 1 5;
      1 0 0 3.2;
      0 0 0 1];


h = 100;
  
traj_T(:,:,1:h/2) = ctraj(T1, T3, h/2);
traj_T(:,:,h/2+1:h) = ctraj(T3, T4, h/2);
qant = q0;

for i=1:h
    [q(i,:), success(i)] = inv_kinematics_kuka_kr_1000(Robot, dh, traj_T(:,:,i), qant, 1);
    qant = q(i,:);
end







