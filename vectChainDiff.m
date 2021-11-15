function [ xdot ] = vectChainDiff( x, vars, dvars)

    for ii = 1:size(x,1)
        xdot(ii,1) = chainDiff(x(ii,1), vars, dvars);
    end

end

