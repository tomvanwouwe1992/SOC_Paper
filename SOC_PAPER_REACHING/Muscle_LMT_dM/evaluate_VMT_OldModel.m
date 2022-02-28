function VMT_est = evaluate_VMT(x,theta,dtheta)
a = x(:,1);
b = x(:,2); 
c = x(:,3);
d = x(:,4);  
h = x(:,5);
VMT_est = - (a.*dtheta + b.*cos(c.*theta + d).*dtheta);


end