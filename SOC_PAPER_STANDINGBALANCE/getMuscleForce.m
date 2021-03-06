function [FMmultiplier,Fp,lMtilde] = getMuscleForce(q,qdot,auxdata)


LMT_coeff = auxdata.LMTcoeff;
dM_coeff = auxdata.dMcoeff;

lMT = LMT_coeff*[1; q];
vMT = LMT_coeff(:,2)*qdot;
dM = dM_coeff*[1; q];

FMo = auxdata.Fiso;
lMo = auxdata.lMopt;
lTs = auxdata.lTs;
alphao = auxdata.alpha;
vMmax = 10;


% Hill-type muscle model: geometric relationships
lM = sqrt((lMo.*sin(alphao)).^2+(lMT-lTs).^2);
lMtilde = lM./lMo;

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



cos_alpha = (lMT-lTs)./lM;
vM = vMT.*cos_alpha;
vMtilde = vM./vMmax;
Fvparam = auxdata.Fvparam;
e1 = Fvparam(1);
e2 = Fvparam(2);
e3 = Fvparam(3);
e4 = Fvparam(4);

% FMvtilde = vMtilde + 1; %e1*log((e2*vMtilde+e3)+sqrt((e2*vMtilde+e3).^2+1))+e4;
FMvtilde = e1*log((e2*vMtilde+e3)+sqrt((e2*vMtilde+e3).^2+1))+e4 ;
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
FM_damp = auxdata.muscleDampingCoefficient.*vMtilde; %0.2*log(1+exp(5*(vMtilde-1)));
FMmultiplier = FMo.*Fce;
Fp = FMo.*(Fpe+FM_damp);
