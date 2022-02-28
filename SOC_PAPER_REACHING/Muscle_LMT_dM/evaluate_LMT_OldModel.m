function LMT_est = evaluate_LMT(x,theta)
a = x(:,1);
b = x(:,2); 
c = x(:,3);
d = x(:,4);  
h = x(:,5);
LMT_est = - (a.*theta + b./c.*sin(c.*theta + d)) + h;


end