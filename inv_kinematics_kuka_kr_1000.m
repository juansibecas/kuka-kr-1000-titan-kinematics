function [q_sol, exito] = inv_kinematics_kuka_kr_1000(Robot, dh, T_dato, q0, mejor)
    %{
        Calcula la cinematica inversa del robot kuka kr 1000 y devuelve el
        vector articular mas cercano a q0, cuando q_mejor = True. En caso
        de que q_mejor = False, devuelve una matriz de tamano MxN, donde
        cada fila es un vector solucion de la cinematica inversa.
        - q_sol es un vector 1xN o una matriz MxN dependiendo del valor de
        el argumento mejor:
                            - mejor == 1 -> q_sol 1xN
                            - mejor == 0 -> q_sol MxN 
        - exito es 1 o 0 si fue exitoso o no el calculo de la cinematica 
        inversa.
        - Robot es un objeto de la clase SerialLink
        - T_dato es la matriz de transformacion homogenea tal que 
        q_sol = f(T_dato), siendo f la cinematica inversa
        - q0 es el vector articular de la posicion actual del robot
    %}
    
    % Obtengo distancias de la matriz DH
    d1 = dh(1,2); %#ok<NASGU>
    a1 = dh(1,3); %#ok<NASGU>
    d2 = -dh(2,3);
    a2 = -dh(3,3);
    d3 = -dh(4,2);
    d4 = dh(6,2);
    
    
    % Se anulan las matrices de base y tool
    T_total = SE3(T_dato);
    T_dato = Robot.base.inv * SE3(T_dato) * Robot.tool.inv;
    
    % En q se van a guardar todas las soluciones, cada fila es una configuracion
    % articular q1 q2 q3 q4 q5 q6
    q = zeros(8,6);

    % Primer problema de Pieper
    p06 = T_dato.t;
    a06 = T_dato.a;
    p04 = p06 - d4*a06;
    xc = p04(1);
    yc = p04(2);
    zc = p04(3); %#ok<NASGU>

    % En plano x0-y0
    % Calculo de q1
    q1 = atan2(yc, xc);
    q(1,1) = q1;
    q(2,1) = q1;
    if q1 > 0
        q1 = q1 - pi;
    else
        q1 = q1 + pi;
    end
    q(3,1) = q1;
    q(4,1) = q1;

    % En el sistema 1
    % Calculo de q2
    % is_real(i) es 1 o 0 si la solucion del vector q(i,:) es exacta o 
    % aproximada respectivamente
    is_real = ones(8,1);
    for i=1:4
        T01 = Robot.A(1 , q(i,1));
        p14 = T01.inv.double * [p04; 1];
        x14 = p14(1);
        y14 = p14(2);

        R1 = sqrt(x14^2+y14^2);

        beta = atan2(y14, x14);

        d3p = sqrt(a2^2+d3^2);

        alpha = acos((-d3p^2+d2^2+R1^2)/(2*d2*R1));
        
        is_real(i) = isreal(alpha);

        if mod(i,2) == 1
            q(i,2) = pi/2 + beta - real(alpha);
        else
            q(i,2) = pi/2 + beta + real(alpha);
        end
    end

    % En el sistema 2
    % Calculo de q3
    for i=1:4
        T02 = Robot.A([1 2] , q(i,1:2));
        p24 = T02.inv.double *[p04; 1]; 
        x24 = p24(1);
        y24 = p24(2);

        % Resuelvo sistema de ecuaciones A*x=b para x=[cos(q3), sen(q3)]
        % Se podria resolver con formula analitica, metodo numerico o A\b
        % (inversa)
        
        A = [-a2 d3;
             -d3 -a2]; %#ok<NASGU>
        b = [x24;
             y24]; %#ok<NASGU>
        
        % Matriz inversa
        %x = A\b;
        
        % Metodo numerico
        %x = cgs(A,b);
        
        % Metodo analitico
        x = [-(a2*x24 + d3*y24)/(a2^2 + d3^2);
             -(a2*y24 - d3*x24)/(a2^2 + d3^2)];
           
        % Se obtiene la solucion de x
        cosq3 = x(1);
        senq3 = x(2);

        q(i,3) = atan2(senq3,cosq3);
    end

    % Copio los valores obtenidos para buscar 2 soluciones diferentes para c/u
    q(5:8,1:3) = q(1:4,1:3);
    is_real(5:8) = is_real(1:4);

    % Segundo problema de pieper
    for i=1:8
        % En el sistema 3
        T03 = Robot.A([1 2 3], q(i, 1:3));
        T36 = T03.inv * T_dato;
        if abs(T36.a(3) - 1) < eps
            % Solucion degenerada:
            % q5 = 0
            % q4 y q6 generan el mismo movimiento
            % q4 = q0(4)
            q4 = q0(4);
            q5 = 0;
            q6 = atan2(T36.n(2), T36.n(1)) - q4;

            q(i,4) = q4;
            q(i,5) = q5;
            q(i,6) = q6;
        else
            % Solucion normal:
            if i <= 4
                q4 = atan2(T36.a(2), T36.a(1));
                q(i,4) = q4;
                % Suma o resta 180 grados para la segunda solucion de q4
                if q4 > 0, q(i+4,4) = q4 - pi; else, q(i+4,4) = q4 + pi; end
            end

            % En el sistema 4
            T34 = Robot.A(4, q(i, 1:4));
            T46 = T34.inv * T36;
            q(i,5) = atan2(T46.a(2) , T46.a(1)) + pi/2;

            % En el sistema 5
            T45 = Robot.A(5, q(i, 1:5));
            T56 = T45.inv * T46;
            q(i,6) = atan2(T56.n(2), T56.n(1));

        end
    end
    
    % Test +- n*2*pi en q4 y q6
    % Como el rango articular de q4 y q6 presenta varias soluciones para un
    % mismo punto, se toma la solucion mas cercana a q0(4) y q0(6) en q4 y q6

    for i=1:8
        q4_b = q(i, 4):-2*pi:Robot.qlim(4,1);
        q6_b = q(i, 6):-2*pi:Robot.qlim(6,1);
        q4_f = q(i, 4):2*pi:Robot.qlim(4,2);
        q6_f = q(i, 6):2*pi:Robot.qlim(6,2);

        % Vectores de posibles angulos q4 y q6
        q4_range = unique([fliplr(q4_b), q4_f]);
        q6_range = unique([fliplr(q6_b), q6_f]);

        % Distancia de cada valor a q0
        q4_dist = abs(q4_range - q0(4));
        q6_dist = abs(q6_range - q0(6));

        % Elijo el mas cercano a q0 (distancia minima)
        q4_dist_min = min(q4_dist);
        q6_dist_min = min(q6_dist);

        % Guardo los indices de los valores a la menor distancia
        % Indexacion logica
        q4_idx = q4_dist == q4_dist_min;
        q6_idx = q6_dist == q6_dist_min;
        
        % Cambio q4 y q6 al valor que esta a menor distancia de q0(4) y
        % q0(6)
        
        % Si hay 2 soluciones a la misma distancia articular, selecciono
        % una sola
        
        [~, n4] = size(q4_range(q4_idx));
        [~, n6] = size(q6_range(q6_idx));
        
        if n4 > 1
            q4_sol = q4_range(q4_idx);
            q(i, 4) = q4_sol(1);
        else
            q(i, 4) = q4_range(q4_idx);
        end
        
        if n6 > 1
            q6_sol = q6_range(q6_idx);
            q(i, 6) = q6_sol(1);
        else
            q(i, 6) = q6_range(q6_idx);
        end
    end

    % limit_check(i) es 1 o 0 si el vector q(i,:) esta dentro o no de los limites
    % articulares respectivamente
    limit_check = ones(8,1);
    for i=1:8
        % Chequea limites articulares
        for j=1:6
            qmin = Robot.qlim(j,1);
            qmax = Robot.qlim(j,2);
            [q(i,j), check] = check_limits(q(i,j), qmin, qmax);
            if check == 0
                limit_check(i) = 0;
            end
        end
    end

    % Calculo distancia articular (entre la config anterior y las soluciones
    % obtenidas) y distancia cartesiana (entre la posicion objetivo y las
    % soluciones obtenidas)
    d_art = zeros(8,1);
    d_xyz = zeros(8,1);
    for i=1:8
        v_xyz = T_total.t - Robot.fkine(q(i,:)).t;
        d_xyz(i) = norm(v_xyz);
        v_art = q(i,:) - q0;
        d_art(i) = norm(v_art);
    end

    % Logica de seleccion de solucion
    j = 1;
    null_flag = 0;
    % Separo las soluciones exactas que se encuentran dentro de limites
    for i=1:8
        if is_real(i) && limit_check(i)
            indexes(j) = i; %#ok<AGROW> % Indices de las soluciones exactas y en limites
            j = j + 1;
            null_flag = 1;
        end
    end

    % n es la cantidad de soluciones exactas y en limites articulares
    if null_flag == 1
        [~, n] = size(indexes);
    else
        n = 0;
    end
    
    d = 1e5;
    % Si hay una o mas de una, selecciono la de menor distancia articular a q0
    if n >= 1
        for i=1:length(indexes) 
            if d_art(indexes(i)) < d
                d = d_art(indexes(i));
                idx = indexes(i);
            end
        end
        q_sol = q(idx,:);
        exito = 1;
    % Si no hay ninguna, selecciono la de menor distancia cartesiana al objetivo
    elseif n == 0
        for i=1:8
            if d_xyz(i)<d
                d = d_xyz(i);
                idx = i;
            end
        end
        q_sol = q(idx,:);
        exito = 0;
    end 
    
    % Devuelve la matriz completa si mejor == 0
    if mejor == 0
        q_sol = q;
    end
    
end

function [v, check] = check_limits(q, qmin, qmax)
    %{
        Chequea si el valor q se encuentra entre qmin y qmax.
        Si q esta dentro del intervalo, v = q y check = 1
        Si q esta fuera del intervalo, v toma el valor del limite mas
        cercano, y check = 0
    %}
    check = 1;
    if q < qmin
        check = 0;
        v = qmin;
    elseif q > qmax
        check = 0;
        v = qmax;
    end
    if check == 1
        v = q;
    end
end