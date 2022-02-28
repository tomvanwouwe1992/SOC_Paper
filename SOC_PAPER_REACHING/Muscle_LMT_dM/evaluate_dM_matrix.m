function dM_matrix = evaluate_dM_matrix(dM_coefficients,theta_shoulder,theta_elbow)

dM_matrix = [evaluate_dM(dM_coefficients(:,1),dM_coefficients(:,2),dM_coefficients(:,3),theta_shoulder) evaluate_dM(dM_coefficients(:,4),dM_coefficients(:,5),dM_coefficients(:,6),theta_elbow)]';