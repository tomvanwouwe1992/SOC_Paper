function LMT = evaluate_LMT(a_shoulder,b_shoulder,c_shoulder,a_elbow,b_elbow,c_elbow,l_base,l_multiplier,theta_shoulder,theta_elbow)

% Returns the NORMALIZED muscle fiber length!
% Note that in this model the tendons have been ignored and that to compute
% the fiber length we do not need LMT-lt_slack

l_full = a_shoulder.*theta_shoulder + b_shoulder.*sin(c_shoulder.*theta_shoulder)./c_shoulder + a_elbow.*theta_elbow + b_elbow.*sin(c_elbow.*theta_elbow)./c_elbow;
LMT = l_full.*l_multiplier + l_base;