function dX = forwardMusculoskeletalDynamics_motorNoise(X,u,T_EXT,wM,auxdata)
a = X(1:6);
q = X(7:8);
qdot = X(9:10);

[Fa,Fp,~,~,~,~,~,~,~] = getMuscleForce(q,qdot,auxdata);

Fm = a.*Fa + Fp;
T = TorqueForceRelation(Fm,q,auxdata) + wM;

F_forceField = auxdata.forceField*(auxdata.l1*cos(q(1,:)) + auxdata.l2*cos(q(1,:)+q(2,:)));
T_forceField = -F_forceField*[auxdata.l2*sin(q(1,:)+q(2,:))+auxdata.l1*sin(q(1,:));auxdata.l2*sin(q(1,:)+q(2,:))];
              

ddtheta = armForwardDynamics(T,q(2),qdot,T_EXT+T_forceField,auxdata);



dX =  [(u-a)./auxdata.tau; qdot; ddtheta];