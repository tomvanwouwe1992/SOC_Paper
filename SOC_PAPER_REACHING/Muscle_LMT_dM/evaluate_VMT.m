function VMT = evaluate_VMT(a_shoulder,b_shoulder,c_shoulder,a_elbow,b_elbow,c_elbow,lM_multiplier,theta_shoulder,theta_elbow,dtheta_shoulder,dtheta_elbow)

% Returns the fiber velocity NORMALIZED by the optimal fiber lenght. The
% units are # optimal fiber lengths per second.

v_full = a_shoulder.*dtheta_shoulder + b_shoulder.*cos(c_shoulder.*theta_shoulder)*dtheta_shoulder + a_elbow.*dtheta_elbow + b_elbow.*cos(c_elbow.*theta_elbow)*dtheta_elbow;
VMT = lM_multiplier.*v_full;