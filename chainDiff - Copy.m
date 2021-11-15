function [ xdot ] = chainDiff( x, vars, dvars )

    xdot = 0;

    for varN = 1:length(vars)
        xdot = xdot + diff(x,vars(varN))*dvars(varN);
    end

end

