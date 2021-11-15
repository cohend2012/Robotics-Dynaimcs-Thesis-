function [ U_B ] = PotentialEnergy( IIRB, mB, ITB, BBGamma, gz )

    U_B = [0 0 gz] * (IIRB * mB + ITB * BBGamma);

end

