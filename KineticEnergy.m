function [ K_B ] = KineticEnergy( IIRBdot, mB, ITBdot, BBGamma, BBJ )

    BBJhat = 0.5 * trace(BBJ) * eye(3) - BBJ;
    
    K_B_translational = 0.5 * transpose(IIRBdot) * IIRBdot * mB;
    
    K_B_coupling = transpose(IIRBdot) * ITBdot * BBGamma;
    
    K_B_rotational = 0.5 * trace(ITBdot * BBJhat * transpose(ITBdot));
    
    K_B = K_B_translational + K_B_coupling + K_B_rotational;

end

