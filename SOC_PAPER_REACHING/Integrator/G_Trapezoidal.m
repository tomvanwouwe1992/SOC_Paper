function G = G_Trapezoidal(X_i,X_i_plus,dX_i,dX_i_plus,dt)

   


G =  X_i_plus - (X_i + (dX_i + dX_i_plus)/2*dt);