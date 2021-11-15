function [ xdot ] = vectPartial( x, vars )

    for ii = 1:size(vars,1)
        xdot(ii,1) = diff(x, vars(ii,1));
    end

end

