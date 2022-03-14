function ddtheta = armForwardDynamics(T,theta_elbow,qdot,T_EXT,auxdata)

ddtheta = T_EXT; 
dtheta_shoulder = qdot(1,:);
dtheta_elbow = qdot(2,:);
a1 = auxdata.I1 + auxdata.I2 + auxdata.m2*auxdata.l1^2;
a2 = auxdata.m2*auxdata.l1*auxdata.lc2;
a3 = auxdata.I2;

for i = 1:size(T,2)
    M = [a1+2*a2*cos(theta_elbow(i))   a3 + a2*cos(theta_elbow(i));
        a3 + a2*cos(theta_elbow(i))   a3];

    C = a2*sin(theta_elbow(i))*[ -dtheta_elbow(i)*(2*dtheta_shoulder(i) + dtheta_elbow(i));
        dtheta_shoulder(i)^2];

    % Joint friction matrix
    B = [0.05 0.025;
        0.025 0.05];

    ddtheta(:,i) = M\(T(:,i) + T_EXT(:,i) - C - B*qdot(:,i));
end
end

