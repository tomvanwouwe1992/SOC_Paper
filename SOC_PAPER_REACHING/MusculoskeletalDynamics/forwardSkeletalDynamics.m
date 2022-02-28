function dX = forwardSkeletalDynamics(X,u,auxdata)

q = X(1:2);
qdot = X(3:4);
T = u;

ddtheta = armForwardDynamics(T,q(2),qdot,auxdata);

dX = [qdot; ddtheta];

