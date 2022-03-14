function T = TorqueForceRelation(Fm,q,auxdata)
theta_shoulder = q(1,:);
theta_elbow = q(2,:);
dM_matrix = evaluate_dM_matrix(auxdata.dM_coefficients,theta_shoulder,theta_elbow);

T = dM_matrix*Fm;
T = [diag(T(1:size(q,2),:))'; diag(T(size(q,2)+1:end,:))'];
end

