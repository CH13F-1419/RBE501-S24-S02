function I_mat = cal_IMat(m, w,h, l)
    Ixx = m* (w^2 + h^2)/12;
    Iyy = m* (l^2 + h^2)/12;
    Izz = m* (l^2 + w^2)/12;
    I_mat = [Ixx 0 0 ;
            0 Iyy 0;
            0 0 Izz];
end


