function I_mat = cal_IMat_c( r,h, m )
    Ixx = m* (3 * r^2 + h^2)/12;
    Iyy = m* (3 * r^2 + h^2)/12;
    Izz = m* r^2 /2; 

    I_mat = [Ixx 0 0 ;
            0 Iyy 0;
            0 0 Izz];
end