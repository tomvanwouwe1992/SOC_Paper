function dX = forwardMusculoskeletalDynamics_monoarticular_motorNoise_OM(X,u,T_EXT,wM,auxdata)
a = X(1:4);
q = X(5:6);
qdot = X(7:8);

[Fa,Fp,~,~,~,~,~,~,~] = getMuscleForce_OldModel(q,qdot,auxdata);

Fm = a.*Fa + Fp;
dM = [evaluate_dM_OldModel(auxdata.dMcoeff(1:2,:),q(1))  evaluate_dM_OldModel(auxdata.dMcoeff(3:4,:),q(2))];
% Joint torques (corrupted by motor noise)
T_shoulder = Fm(1:2)'*dM(:,1) + wM(1);
T_elbow =   Fm(3:4)'*dM(:,2) + wM(2);


armEndPoint = p2(auxdata.l1,auxdata.l2,q(1),q(2));
Fx = auxdata.FV*armEndPoint(1);
Fy = 0;

ddtheta = explicitSkeletalDynamics(Fx,Fy,auxdata.I1,auxdata.I2,T_elbow,T_shoulder,T_EXT,qdot(1),qdot(2),auxdata.l1,auxdata.l2,auxdata.lc1,auxdata.lc2,auxdata.m1,auxdata.m2,q(1),q(2));


dX =  [(u-a)./auxdata.tau; qdot; ddtheta];