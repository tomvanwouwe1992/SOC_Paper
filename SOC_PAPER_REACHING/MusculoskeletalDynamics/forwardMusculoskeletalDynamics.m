function dX = forwardMusculoskeletalDynamics(X,u,auxdata)
a = X(1:6);
q = X(7:8);
qdot = X(9:10);

[Fa,Fp,~,~,~,~,~,~,~] = getMuscleForce(q,qdot,auxdata);

Fm = a.*Fa + Fp;
T = TorqueForceRelation(Fm,q,auxdata);
ddtheta = armForwardDynamics(T,q(2),qdot,[0;0],auxdata);

dX =  [(u-a)./auxdata.tau; qdot; ddtheta];
