function [R, dh, W, STLPath] = robot()
    %{
        Devuelve las variables del robot KUKA KR 1000 1300 TITAN
        - R es un objeto de la clase SerialLink
        - dh es la matriz de parametros Denavit-Hartenberg
        - W es el espacio de trabajo
        - STLPath es la direccion en la que se ubican los modelos STL del
        robot
    %}

    % Parámetros DH acordes al modelo STL
    dh = [0.000  1.100  0.600  -pi/2  0;
          0.000  0.000  -1.400  0.000 0;
          0.000  0.000  -0.065  -pi/2  0;
          0.000  -1.200  0.000  pi/2 0;
          0.000  0.000  0.000  -pi/2  0;
          0.000  0.372  0.000  0.000  0];

    R = SerialLink(dh,'name','KR 1000 1300 TITAN');

    % Límites articulares y offsets para adecuar el robot a una posición de
    % home con los datos obtenidos del fabricante

    R.base = transl(0, 0, 1);

    R.qlim(1,1:2) = [-150,  150]*pi/180;
    R.qlim(2,1:2) = [-40,  107.5]*pi/180;
    %R.qlim(2,1:2) = [-180, 180]*pi/180;  % Prueba para ikine
    R.qlim(3,1:2) = [-28, 45]*pi/180;
    %R.qlim(3,1:2) = [-180, 180]*pi/180;  % Prueba para ikine
    R.qlim(4,1:2) = [-1000,  1000]*pi/180;
    R.qlim(5,1:2) = [-118,  118]*pi/180;
    R.qlim(6,1:2) = [-350,  350]*pi/180;

    % Extremo del efector final a x=-0.5m, z=2.5m
    R.tool = transl(-0.5, 0, 2.5);

    R.offset = [0, 90, 0, 0, 180, 0]*pi/180;


    % Espacio de Trabajo
    W = [-7, 7, -7, 7, -1, 5];
    
    % Path de los modelos STL
    STLPath = strcat(pwd, '\KR1000_1300_TITAN');
