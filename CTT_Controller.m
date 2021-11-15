function [ u ] = CTT_Controller( g, gDes, gd, gdDes, gddDes, Kp, Kd )

    u = gddDes + Kd*(gdDes - gd) + Kp*(gDes - g);

end

