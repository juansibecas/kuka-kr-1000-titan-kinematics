function T = rotate_from_transl(x, y, z)
%{
    Returns all T matrixes with X pointing upwards and position (x,y,z)
%}
    h = 90;
    T = zeros(4, 4, h);
    for i=1:h
        ang = pi/2*(i/h);
        T(:, :, i) = [0     sin(ang)    cos(ang)    x;
                      0     -cos(ang)   sin(ang)    y;
                      1     0           0           z;
                      0     0           0           1];
    end
end