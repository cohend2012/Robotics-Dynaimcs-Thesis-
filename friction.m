function [ Ff ] = friction( Fr, mus, muk, Nz )

    Ff = (1 - sigmoid(Fr - mus.*Nz, 10)).*Fr + sigmoid(Fr - mus.*Nz, 10).*muk.*Nz;

end

