function [ out ] = saturate( in, low, high )

    out = in;

    for ii = 1:length(in)
        if in(ii) > high
            out(ii) = high;
        elseif in(ii) < low
            out(ii) = low;
        end
    end

end

