function [] = esp_trab(plot_xz, plot_xy)
    %{
        Plotea el limite del espacio de trabajo en los planos XY, XZ
        - Si plot_xz == 1, plotea XZ 
        - Si plot_xy == 1, plotea XY
        Se guarda la curva a graficar en la matriz p(1:3, t0) = [x0, y0, z0]
        Cada posicion se obtiene con fkine(qi) variando qi en intervalos
        especificos
    %}

    [R, ~, W, STLPath] = robot();
    
    q = zeros(1,6);
    v = zeros(1,7);
    STL = 1;
    teach = 1;
    robotplot(R, W, q, v, STL, STLPath, teach);
    k = 600;
    
    if plot_xz == 1
        
        % Curva 1
        q0 = [0, 107.5, 11, 0, -118, 0]*pi/180;
        qf = [0, 107.5, -28, 0, -80, 0]*pi/180;
        [q1, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 2
        q0 = [0, 107.5, -28, 0, -80, 0]*pi/180;
        qf = [0, -40, -28, 0, 67, 0]*pi/180;
        [q2, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 3
        q0 = [0, -40, -28, 0, 67, 0]*pi/180;
        qf = [0, -40, 45, 0, -5, 0]*pi/180;
        [q3, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 4
        q0 = [0, -40, 45, 0, -5, 0]*pi/180;
        qf = [0, 107.5, 11, 0, -118, 0]*pi/180;
        [q4, ~, ~ ]= jtraj(q0, qf, k);
        
        q = [q1;
             q2;
             q3;
             q4];
         
        [m, n] = size(q);
        % En pxz se guardan los puntos de la curva para graficar
        pxz = zeros(m, n);
        
        for i = 1:m
            T = R.fkine(q(i,:));
            pxz(1:3, i) = T.t;
        end
        
        plot3(pxz(1,:),pxz(2,:),pxz(3,:), 'LineWidth', 3 )
    end

    if plot_xy == 1
        
        % Curva 1
        q0 = [-150, 57.8, -27.2, 0, -30.8, 0]*pi/180;
        qf = [150, 57.8, -27.2, 0, -30.8, 0]*pi/180;
        [q1, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 2
        q0 = [150, 57.8, -27.2, 0, -30.8, 0]*pi/180;
        qf = [150, -14.3, 45, 0, -30.8, 0]*pi/180;
        [q2, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 3
        q0 = [150, -14.3, 45, 0, -30.8, 0]*pi/180;
        qf = [-150, -14.3, 45, 0, -30.8, 0]*pi/180;
        [q3, ~, ~ ]= jtraj(q0, qf, k);
        % Curva 4
        q0 = [-150, -14.3, 45, 0, -30.8, 0]*pi/180;
        qf = [-150, 57.8, -27.2, 0, -30.8, 0]*pi/180;
        [q4, ~, ~ ]= jtraj(q0, qf, k);
        
        q = [q1;
             q2;
             q3;
             q4];
         
        [m, n] = size(q);
        % En pxy se guardan los puntos de la curva para graficar
        pxy = zeros(m, n);
        
        for i = 1:m
            T = R.fkine(q(i,:));
            pxy(1:3, i) = T.t;
        end
        
        plot3(pxy(1,:), pxy(2,:), pxy(3,:), 'LineWidth', 3 )
    end
end
