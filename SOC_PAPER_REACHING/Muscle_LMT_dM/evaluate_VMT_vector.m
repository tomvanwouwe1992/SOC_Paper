function VMT_vector = evaluate_VMT_vector(dM_coefficients,LMT_coefficients,theta_shoulder,theta_elbow,dtheta_shoulder,dtheta_elbow)

VMT_vector = evaluate_VMT(dM_coefficients(:,1),dM_coefficients(:,2),dM_coefficients(:,3),dM_coefficients(:,4),dM_coefficients(:,5),dM_coefficients(:,6),LMT_coefficients(:,2),theta_shoulder,theta_elbow,dtheta_shoulder,dtheta_elbow);
