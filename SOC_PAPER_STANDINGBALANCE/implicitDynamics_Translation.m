function [err] = implicitDynamics_Translation(a_base,a,qA,qAdot,K,u,wM,wPqdotdot,wPq,wPqdot,wVq,wVqdot,auxdata)

[FMmultiplier, Fp, lMtilde] = getMuscleForce(qA,qAdot,auxdata);

% Feedback control with upright quiet stance as reference position
% determines the excitation of the torque actuator
e_prop = K(1:2,:)*([qA; qAdot] + [wPq;wPqdot]);

e_vest = K(3:4,:)*([qA; qAdot] + [wVq;wVqdot]);

e = e_prop + e_vest;
% Parameters of the "skeletal" system
m = auxdata.m; I = auxdata.I; l = auxdata.l; g = auxdata.g; Topt = auxdata.Topt;

dM = auxdata.dMcoeff*[1; qA];
FM = (a_base + a + wM).*FMmultiplier + Fp;

FMo = auxdata.Fiso;
F_SRS = auxdata.SRS_coeff*a_base.*FMo./auxdata.lMopt.*(lMtilde - auxdata.lMtilde_Upright);

M_E = (FM+F_SRS)'*dM;
% Generalized accelerations of the "skeletal" sytem
qAdotdot = (m*g*l*sin(qA) + M_E - (m*l^2+I)*wPqdotdot)/(m*l^2+I);

adot = (e - a)/auxdata.tau;

err =  [ adot - u(1:2);
         qAdot - u(3);
         qAdotdot - u(4)];

end