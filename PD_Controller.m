function [ out, err_] = PD_Controller( e, Kp, Kd, dt, err )

    err_ = e;
    errDrv = (err_ - err)/dt;
    
    out = Kp*err_ + Kd*errDrv;

end

