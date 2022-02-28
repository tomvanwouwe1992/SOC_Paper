function dM = evaluate_dM(a,b,c,theta)
% Returns the muscle moment arm
dM = a + b.*cos(c.*theta);