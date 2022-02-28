function [X_stoch,EE_stoch] = stochasticForwardSim_4muscles_OldModel(X_init,e_ff,EE_ref,K,auxdata,functions,perturbation)
import casadi.*;

e_ff_MX = MX.sym('u_MX',4);       % Controls (muscle excitations - feedforward)
X_MX = MX.sym('X_MX',auxdata.nStates); % States (muscle activations and joint kinematics)
wM_MX = MX.sym('wM_MX',2);     % Motor noise
wPq_MX = MX.sym('wPq_MX',2);
wPqdot_MX = MX.sym('wPqdot_MX',2);
F_EE_MX = MX.sym('F_EE_MX',2);
EE_ref_MX = MX.sym('EE_ref_MX', 4);
EE_MX = [EndEffectorPos(X_MX(5:6),auxdata); EndEffectorVel(X_MX(5:6),X_MX(7:8),auxdata)];
K_MX = MX.sym('K_MX',4,4);
e_fb_MX = K_MX*((EE_MX - EE_ref_MX) + [wPq_MX;wPqdot_MX]);
u_MX = e_ff_MX + e_fb_MX;

dX_MX = forwardMusculoskeletalDynamics_monoarticular_motorNoise_OM(X_MX,u_MX,F_EE_MX,wM_MX,auxdata);
f_forwardMusculoskeletalDynamics = Function('f_forwardMusculoskeletalDynamics',{X_MX,e_ff_MX,K_MX,EE_ref_MX,F_EE_MX,wM_MX,wPq_MX,wPqdot_MX},{dX_MX});  

% Noise samples
R = mvnrnd(zeros(6,1),(auxdata.sigma_w),auxdata.N+1)';
R_prev = R;
for i = 1:auxdata.N+1
    R(:,i) = min(2*sqrt(diag(auxdata.sigma_w)),R(:,i));  R(:,i) = max(-2*sqrt(diag(auxdata.sigma_w)),R(:,i)); 
end


Urf = MX.sym('Urf',auxdata.nStates*2);
X_i = X_init;
X_stoch = NaN(auxdata.nStates,auxdata.N+1); X_stoch(:,1) = X_init;
EE_stoch = NaN(4,auxdata.N+1); EE_stoch(:,1) = [EndEffectorPos(X_init(5:6),auxdata); EndEffectorVel(X_init(5:6),X_init(7:8),auxdata)];

% T_EE = zeros(2,auxdata.N+1);
% T_EE(:,27:27+5) = repmat(perturbation,1,6);
pertTime = 0;
for i = 1:auxdata.N
    K_i = reshape(K(:,i),4,4);
    K_i_plus = reshape(K(:,i+1),4,4);
    if EE_stoch(2,i) > 0.279 && pertTime < 5
        T_EE = perturbation;
        pertTime = pertTime + 1;
    else
        T_EE = 0*perturbation;
    end
    EE_ref_i = EE_ref(:,i); EE_ref_i_plus = EE_ref(:,i+1);
    T_EE_i = T_EE; T_EE_i_plus = T_EE;
    dX_i = f_forwardMusculoskeletalDynamics(X_i,e_ff(:,i),K_i,EE_ref_i,T_EE_i,R(1:2,i),R(3:4,i),R(5:6,i));
    rf = rootfinder('rf','newton',struct('x',Urf,'g',[Urf(1:8) - (X_i + (dX_i + Urf(9:16))/2*auxdata.dt); ...
                                                      Urf(9:16) - f_forwardMusculoskeletalDynamics(Urf(1:8),e_ff(:,i+1),K_i_plus,EE_ref_i_plus,T_EE_i_plus,R(1:2,i+1),R(3:4,i+1),R(5:6,i+1))]),struct('abstol',1e-16));
    solution = rf([X_i;dX_i],[]);
    X_i = full(solution(1:8));
    X_stoch(:,i+1) = X_i;
    EE_stoch(:,i+1) = [EndEffectorPos(X_i(5:6),auxdata); EndEffectorVel(X_i(5:6),X_i(7:8),auxdata)];
end


