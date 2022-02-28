function [dM_est,LMT_est] = evaluate_dM(x,theta)
a = x(:,1);
b = x(:,2); 
c = x(:,3);
d = x(:,4);  
h = x(:,5);
dM_est = a + b.*cos(c.*theta + d);
end