function T = TorqueForceRelation(Fm,q,auxdata)
theta_shoulder = q(1,:);
theta_elbow = q(2,:);
dM_matrix = evaluate_dM_matrix(auxdata.dM_coefficients,theta_shoulder,theta_elbow);

T = dM_matrix*Fm;

end

