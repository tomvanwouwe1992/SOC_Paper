function DG_DZ = DG_DZ_Trapzoidal(DdZ_DX,dt)

DG_DZ = eye(size(DdZ_DX,1)) - DdZ_DX*dt/2;