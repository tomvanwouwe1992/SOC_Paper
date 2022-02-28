function ddtheta = armForwardDynamics(T,theta_elbow,qdot,T_EXT,auxdata)
dtheta_shoulder = qdot(1,:);
dtheta_elbow = qdot(2,:);
a1 = auxdata.I1 + auxdata.I2 + auxdata.m2*auxdata.l1^2;
a2 = auxdata.m2*auxdata.l1*auxdata.lc2;
a3 = auxdata.I2;

M = [a1+2*a2*cos(theta_elbow)   a3 + a2*cos(theta_elbow);
     a3 + a2*cos(theta_elbow)   a3];
 
C = a2*sin(theta_elbow)*[ -dtheta_elbow*(2*dtheta_shoulder + dtheta_elbow); 
                           dtheta_shoulder^2];

% Joint friction matrix
B = [0.05 0.025;
     0.025 0.05];
 
ddtheta = M\(T + T_EXT - C - B*qdot);
end

