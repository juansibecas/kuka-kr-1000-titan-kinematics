function [] = robotplot(R, W, q, v, STL, STLPath, teach)
    %{
        Plotea los eslabones del robot y, segun requerido, los sistemas DH
        de cada articulacion.
        - R es un objeto de la clase SerialLink con N articulaciones
        - q es el vector de N coordenadas articulares que se quiere graficar
        - W es el espacio de trabajo
        - v es un vector de tamano N+1 donde cada elemento 1 
        determina si se debe graficar el sistema DH correspondiente a su
        indice. Ej: v = [1, 1, 0, 0, 1, 0, 1] grafica DH 0, 1, 4 y 6
        - STL = 1 para plotear con los modelos STL ubicados en STLPath
        - teach = 1 para plotear con teach
    %}
    
    N = length(R.links);
    
    if or(length(v) ~= N+1, length(q) ~= N)
        error('tamanos de v y q no corresponden')
    end
    
    hold on
    if STL == 1
        R.plot3d(q, 'notiles', 'noarrow', 'path', STLPath, 'workspace', W, 'alpha', 0.1);
    else
        R.plot(q, 'workspace', W);
    end
    
    if teach == 1
        R.teach(q);
    end
    
    % colores que usa por defecto matlab en cada eslabon
    colors = ['#0072BD'; '#D95319'; '#EDB120'; '#7E2F8E'; '#77AC30'; '#4DBEEE'; '#A2142F'];
    for i=1:N+1
        evaluate = v(i);
        if evaluate == 1
            if i == 1
                T = eye(4);
            elseif i == N+1
                T = R.base * R.A(1:i-1, q) * R.tool;
            else
                T = R.base * R.A(1:i-1, q);
            end
            trplot(T, 'noarrow', 'frame', strcat('Eje ', string(i-1)), 'color', colors(i,:), 'length', 2);
        end
    axis(W);
    end       