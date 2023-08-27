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

[q1, exito1] = inv_kinematics_kuka_kr_1000(Robot, dh, T1, q0, 1);

[q2, exito2] = inv_kinematics_kuka_kr_1000(Robot, dh, T2, q1, 1);

[q3, exito3] = inv_kinematics_kuka_kr_1000(Robot, dh, T3, q2, 1);

[q4, exito4] = inv_kinematics_kuka_kr_1000(Robot, dh, T4, q3, 1);
  
h = 100;
  
traj_T(:,:,1:h/2) = ctraj(T1, T3, h/2);
traj_T(:,:,h/2+1:h) = ctraj(T3, T4, h/2);

traj_xyz = zeros(3,h);

for i=1:h
    traj_xyz(1:3, i) = traj_T(1:3, 4, i);
end


qsol(1, :, 1) = q1;

for i=2:h
    x = traj_xyz(1, i);
    y = traj_xyz(2, i);
    z = traj_xyz(3, i);
    
    T = rotate_from_transl(x, y, z);
    n = length(T(1 , 1, :));
    k = 0;
    qant = qsol(1, :, i-1);
    for j=1:n
        [q, success] = inv_kinematics_kuka_kr_1000(Robot, dh, T(:, :, j), qant, 1);
        
        if success == 1
            k = k+1;
            qsol(k,:,i) = q;
        end
    end
end

q = zeros(h,6);
q(1,:) = q1;
for i=2:h
    q(i, :) = get_closest_qnext(q(i-1, :), qsol(:, :, i));
end


function qc = get_closest_qnext(qant, qsol)
    
    [m, ~] = size(qsol);
    dmin = 1e20;
    for i=1:m
        if norm(qsol(i,:)) == 0
           break;
        end
        d = norm(qant - qsol(i, :)); % Euclidian or Manhattan distance?
        if d < dmin
            dmin = d;
            idx = i;
        end
    end
    qc = qsol(idx, :);

end


