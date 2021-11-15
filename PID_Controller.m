function [ out, err_, errInt_ ] = PID_Controller( e, Kp, Ki, Kd, dt, err, errInt )

    err_ = e;
    errInt_ = errInt + err_*dt;
    errDrv = (err_ - err)/dt;
    
    out = Kp*err_ + Ki*errInt_ + Kd*errDrv;

end

