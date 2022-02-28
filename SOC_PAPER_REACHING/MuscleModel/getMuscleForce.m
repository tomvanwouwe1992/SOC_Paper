function [Fa,Fp,lMtilde,vMtilde,FMltilde,FMvtilde,Fce,Fpe,Fpv] = getMuscleForce(q,qdot,auxdata)

% Fa: active muscle force [N]
% Fp: passive muscle force [N]
% lMtilde: normalized fiber lenght [-]
% vMtilde: # optimal fiber lenghts per second at which muscle is lengthening or shortening [-]
% FMltilde: force-length multiplier [-]
% FMvtilde: force-velocity multiplier [-]
% Fce: Active force multiplier [-]
% Fpe: Passive elastic force multiplier [-]
% Fm: Passive viscous force multiplier [-]


% Compute muscle fiber length and muscle fiber velocity 
lMtilde = evaluate_LMT_vector(auxdata.dM_coefficients,auxdata.LMT_coefficients,q(1,:),q(2,:)); % note that LMT = fiber length in the implemented model (no tendon)
vMtilde = evaluate_VMT_vector(auxdata.dM_coefficients,auxdata.LMT_coefficients,q(1,:),q(2,:),qdot(1,:),qdot(2,:)); % note that VMT = fiber velocity in the implemented model (no tendon)
vMtilde_normalizedToMaxVelocity = vMtilde./auxdata.vMtilde_max; % normalize to max number of optimal fiber lengths per second

FMo = auxdata.Fiso;

% Active muscle force-length characteristic
Faparam = auxdata.Faparam;
b11 = Faparam(1);
b21 = Faparam(2);
b31 = Faparam(3);
b41 = Faparam(4);
b12 = Faparam(5);
b22 = Faparam(6);
b32 = Faparam(7);
b42 = Faparam(8);

b13 = 0.1;
b23 = 1;
b33 = 0.5*sqrt(0.5);
b43 = 0;
num3 = lMtilde-b23;
den3 = b33+b43*lMtilde;
FMtilde3 = b13*exp(-0.5*num3.^2./den3.^2);

num1 = lMtilde-b21;
den1 = b31+b41*lMtilde;
FMtilde1 = b11*exp(-0.5*num1.^2./den1.^2);

num2 = lMtilde-b22;
den2 = b32+b42*lMtilde;
FMtilde2 = b12*exp(-0.5*num2.^2./den2.^2);

FMltilde = FMtilde1+FMtilde2+FMtilde3;

% Active muscle force-velocity characteristic
Fvparam = auxdata.Fvparam;
e1 = Fvparam(1);
e2 = Fvparam(2);
e3 = Fvparam(3);
e4 = Fvparam(4);

FMvtilde = e1*log((e2*vMtilde_normalizedToMaxVelocity+e3)+sqrt((e2*vMtilde_normalizedToMaxVelocity+e3).^2+1))+e4 ;

% Active muscle force
Fce = FMltilde.*FMvtilde;

% Passive muscle force-length characteristic
Fpparam = auxdata.Fpparam;

e0 = 0.6;
kpe = 4;
t5 = exp(kpe * (lMtilde - 0.10e1) / e0);
Fpe = ((t5 - 0.10e1) - Fpparam(1)) / Fpparam(2);

% Muscle force + damping 
% Muscle force
Fpv = auxdata.muscleDampingCoefficient.*vMtilde_normalizedToMaxVelocity; 
Fa = FMo.*Fce;
Fp = FMo.*(Fpe+Fpv);
