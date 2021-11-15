function [ out ] = wrap( in, low, high )
    for ii = 1:length(in)
        while (in(ii) > high)
            in(ii) = in(ii) - high + low;
        end
        while (in(ii) < low)
            in(ii) = in(ii) + high - low;
        end
    end
    out = in;
end

