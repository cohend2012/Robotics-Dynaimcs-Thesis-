function [ y ] = sigmoid( x, k)

    y = 1 ./ (1 + exp(-k .* x));

end

