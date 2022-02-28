function DG_DX = DG_DX_Trapezoidal(DdX_DX,dt)

DG_DX = - (DdX_DX*dt/2 + eye(size(DdX_DX,1)));