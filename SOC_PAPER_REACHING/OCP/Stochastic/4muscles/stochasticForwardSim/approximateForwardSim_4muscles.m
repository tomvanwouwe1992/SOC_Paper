function [X, M, EE_ref, Pmat] = approximateForwardSim_4muscles(X_init,Pmat_init,e_ff,K,auxdata,functions)
import casadi.*;
Urf = MX.sym('Urf',auxdata.nStates*2);
X_i = X_init;
X = NaN(auxdata.nStates,auxdata.N+1); X(:,1) = X_init;
EE_ref = NaN(4,auxdata.N); EE_ref(:,1) = [EndEffectorPos(X_i(5:6),auxdata); EndEffectorVel(X_i(5:6),X_i(7:8),auxdata)];

f_forwardMusculoskeletalDynamics = functions.f_forwardMusculoskeletalDynamics;
for i = 1:auxdata.N
    dX_i = f_forwardMusculoskeletalDynamics(X_i,e_ff(:,i),0,0,0,0,0);
    rf = rootfinder('rf','newton',struct('x',Urf,'g',[Urf(1:8) - (X_i + (dX_i + Urf(9:16))/2*auxdata.dt); ...
                                                      Urf(9:16) - f_forwardMusculoskeletalDynamics(Urf(1:8),e_ff(:,i+1),0,0,0,0,0)]),struct('abstol',1e-16));
    solution = rf([X_i;dX_i],[]);
    X_i = full(solution(1:8));
    X(:,i+1) = X_i;
    EE_ref(:,i+1) = [EndEffectorPos(X_i(5:6),auxdata); EndEffectorVel(X_i(5:6),X_i(7:8),auxdata)];
end

M = NaN(auxdata.nStates,auxdata.nStates*auxdata.N);
Pmat_i = Pmat_init;
Pmat = NaN(auxdata.nStates,auxdata.nStates,auxdata.N+1); Pmat(:,:,1) = Pmat_i;
for i = 1:auxdata.N
%     
%     K_i = reshape(K(:,floor(i/10)+1),6,4);
%     K_i_plus = reshape(K(:,floor((i+1)/10)+1),6,4);
    K_i = reshape(K(:,i),4,4);
    K_i_plus = reshape(K(:,i+1),4,4);
    
    DdX_DX_i = functions.f_DdX_DX(X(:,i),e_ff(:,i),K_i,EE_ref(:,i),auxdata.wM,auxdata.wPq,auxdata.wPqdot);
    DdZ_DX_i = functions.f_DdX_DX(X(:,i+1),e_ff(:,i+1),K_i_plus,EE_ref(:,i+1),auxdata.wM,auxdata.wPq,auxdata.wPqdot);
    DdX_Dw_i = functions.f_DdX_Dw(X(:,i),e_ff(:,i),K_i,EE_ref(:,i),auxdata.wM,auxdata.wPq,auxdata.wPqdot);
    
    DG_DX_i = functions.f_DG_DX(DdX_DX_i);
    DG_DZ_i = functions.f_DG_DZ(DdZ_DX_i);
    DG_DW_i = functions.f_DG_DW(DdX_Dw_i);
    M_i = DG_DZ_i^(-1);
    M(:,(i-1)*auxdata.nStates + 1:i*auxdata.nStates) = full(M_i);
    Pmat_i = full(M_i*(DG_DX_i*Pmat_i*DG_DX_i' + DG_DW_i*auxdata.sigma_w*DG_DW_i')*M_i'); % + dGdW*sigmaW*dGdW'
    Pmat(:,:,i+1) = Pmat_i;
end
