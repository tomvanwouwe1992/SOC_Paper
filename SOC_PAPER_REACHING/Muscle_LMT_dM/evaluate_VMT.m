function VMT = evaluate_VMT(a_shoulder,b_shoulder,c_shoulder,a_elbow,b_elbow,c_elbow,lM_multiplier,theta_shoulder,theta_elbow,dtheta_shoulder,dtheta_elbow)

% Returns the fiber velocity NORMALIZED by the optimal fiber lenght. The
% units are # optimal fiber lengths per second.
nCoeff = size(a_shoulder,1);
v_full = a_shoulder*dtheta_shoulder + b_shoulder.*cos(c_shoulder*theta_shoulder).*repmat(dtheta_shoulder,nCoeff,1) + a_elbow*dtheta_elbow + b_elbow.*cos(c_elbow*theta_elbow).*repmat(dtheta_elbow,nCoeff,1);
VMT = lM_multiplier.*v_full;