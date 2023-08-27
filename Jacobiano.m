clear
clc

[Robot, dh, W, STLPath] = robot();

% Punto 1

n_examples = 100;
q_n = zeros(n_examples, 6);
det_J = zeros(n_examples, 1);
tol = 0.01;


for i=1:n_examples
    % Creo configuraciones aleatorias
    for j=1:6
        qmin = Robot.qlim(j,1);
        qmax = Robot.qlim(j,2);
        q_n(i,j) = qmin + rand *(qmax-qmin);
    end
    
    % Fijo angulos (q3 = 0, q5=0, q3 = -q2, q3 = q2)
    q_n(i, 5) = 0;
    
    % Calculo el jacobiano para cada una
    J = Robot.jacob0(q_n(i,:));
    det_J(i) = det(J);   
end

% Ordeno la matriz por el jacobiano de menor a mayor
[det_J, I_1] = sort(det_J, 'ComparisonMethod', 'abs');
q_filt = q_n(I_1,:);

% Separo las configuraciones cuyo determinante del jacobiano son menores a
% la tolerancia especificada
I_2 = find(abs(det_J) < tol);
det_J_2 = det_J(I_2,:);
q_2 = q_filt(I_2,:);

% Ploteo las configuraciones de menores determinantes de jacobiano
[m, ~] = size(q_2);
plots = 5;
if plots > m
    plots = m;
end
for i=1:plots
    q_plot = q_2(i,:);
    disp('espacio para continuar')
    pause()
    Robot.plot3d(q_plot, 'noarrow', 'workspace', W, 'path', STLPath);
end






