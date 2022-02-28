
function out = OCP_uprightStanding_Rotation(Misc,wM,wqPdotdot,wPq,wPqdot,wVq,wVqdot,VESTIBULARLOSS,guess,SRS)

%% ---------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% PART I: INPUTS FOR OPTIMAL CONTROL PROBLEM ---------------------------- %
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
% Check for optional input arguments ------------------------------------ %


% Mesh Frequency
if ~isfield(Misc,'Mesh_Frequency') || isempty(Misc.Mesh_Frequency)
    Misc.Mesh_Frequency=100;
end


% Input arguments
auxdata.Ndof=1;
auxdata.scaling.udx = 100;          % Scaling factor: derivative muscle-tendon force
auxdata.scaling.udw = 0.01;          % Scaling factor: derivative muscle-tendon force
auxdata.scaling.udwS = 100;  
auxdata.w1 = 1000;                      % Weight objective function
auxdata.w2 = 0.01;                       % Weight objective function
auxdata.Topt = 150;                     % Scaling factor: reserve actuators
auxdata.tau = 0.150;
P_init = 0.001^2;

auxdata.m = 70;
auxdata.l = 1;
auxdata.g  = 9.81;
auxdata.I = 0;

auxdata.LMTcoeff = [ 0.2931 0.0508; 0.3035 -0.0447];
auxdata.dMcoeff = [ -0.0508 0.0223; 0.0447 0.0119];
auxdata.Fiso = [ 5137; 3000];
auxdata.lTs = [0.2514; 0.2228];
% auxdata.lMopt = [0.0901; 0.0503; 0.0979];
auxdata.lMopt = 1.05*[ 0.0503; 0.0979];

auxdata.alpha = [ 0.4363; 0.0873];
auxdata.muscleDampingCoefficient = [ 0.01; 0.01];

auxdata.SRS_coeff = SRS;

% Parameters of active muscle force-velocity characteristic
load('ActiveFVParameters.mat','ActiveFVParameters');
Fvparam(1) = 1.475*ActiveFVParameters(1); Fvparam(2) = 0.25*ActiveFVParameters(2);
Fvparam(3) = ActiveFVParameters(3) + 0.75; Fvparam(4) = ActiveFVParameters(4) - 0.027;
auxdata.Fvparam = Fvparam;

% Parameters of active muscle force-length characteristic
load('Faparam.mat','Faparam');
auxdata.Faparam = Faparam;

% Parameters of passive muscle force-length characteristic
e0 = 0.6; kpe = 4; t50 = exp(kpe * (0.2 - 0.10e1) / e0);
pp1 = (t50 - 0.10e1); t7 = exp(kpe); pp2 = (t7 - 0.10e1);
auxdata.Fpparam = [pp1;pp2];
[FMmultiplier, Fp, lMtilde] = getMuscleForce(0,0,auxdata);
auxdata.lMtilde_Upright = lMtilde;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% Generate CasADi functions %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nStates = 4;
import casadi.*

% Implict dynamics
a_MX = MX.sym('a_MX',2);
a_base_MX = MX.sym('a_base_MX',2);
qA_MX = MX.sym('qA_MX',1);
qAdot_MX = MX.sym('qAdot_MX',1);
qPdotdot_MX = MX.sym('qPdotdot_MX',1);
K_MX = MX.sym('K_MX',2*2,2);
u_MX = MX.sym('u_MX',4);
wM_MX = MX.sym('wM_MX',1);
wqPdotdot_MX = MX.sym('wqPdotdot_MX',1);
wPq_MX = MX.sym('wPq_MX',1);
wPqdot_MX = MX.sym('wPqdot_MX',1);

wVq_MX = MX.sym('wVq_MX',1);
wVqdot_MX = MX.sym('wVqdot_MX',1);


dynamicsError = implicitDynamics_Translation(a_base_MX,a_MX,qA_MX,qAdot_MX,K_MX,u_MX,wM_MX,wqPdotdot_MX,wPq_MX,wPqdot_MX,wVq_MX,wVqdot_MX,auxdata);
f_dynamicsError = Function('f_dynamicsError',{a_base_MX,a_MX,qA_MX,qAdot_MX,K_MX,u_MX,wM_MX,wqPdotdot_MX,wPq_MX,wPqdot_MX,wVq_MX,wVqdot_MX},{dynamicsError});

% Derivative of the implicit dynamics to the state
udx_MX = MX.sym('udx_MX',nStates,nStates)
dynamicsDerivativeError = jacobian(dynamicsError, [a_MX' qA_MX qAdot_MX]) + jacobian(dynamicsError, [ u_MX])*udx_MX;
f_dynamicsDerivativeError = Function('f_dynamicsDerivativeError',{a_base_MX,a_MX,qA_MX,qAdot_MX,K_MX,u_MX,wM_MX,wqPdotdot_MX,wPq_MX,wPqdot_MX,wVq_MX,wVqdot_MX,udx_MX},{dynamicsDerivativeError});

% Derivative of the implicit dynamics to the gaussian disturbance (motor noise)
udw_MX = MX.sym('udx_MX',nStates,6)
dynamicswMDerivativeError = jacobian(dynamicsError, [wM_MX wqPdotdot_MX wPq_MX wPqdot_MX wVq_MX wVqdot_MX]) + jacobian(dynamicsError, [ u_MX])*udw_MX;
f_dynamicswMDerivativeError = Function('f_dynamicswMDerivativeError',{a_base_MX,a_MX,qA_MX,qAdot_MX,K_MX,u_MX,wM_MX,wqPdotdot_MX,wPq_MX,wPqdot_MX,wVq_MX,wVqdot_MX,udw_MX},{dynamicswMDerivativeError});

dynamicsError = implicitDynamics_Translation(a_base_MX,a_MX,0*qA_MX,0*qAdot_MX,0*K_MX,u_MX,0*wM_MX,0*wqPdotdot_MX,0*wPq_MX,0*wPqdot_MX,0*wVq_MX,0*wVqdot_MX,auxdata);

constr_par = [vertcat(dynamicsError{4}); vertcat(dynamicswMDerivativeError{:}); vertcat(dynamicsDerivativeError{:})];

f_coll = Function('f_coll',{a_base_MX,a_MX,qA_MX,qAdot_MX,K_MX,u_MX,wM_MX,wqPdotdot_MX,wPq_MX,wPqdot_MX,wVq_MX,wVqdot_MX,udx_MX,udw_MX},{constr_par}); %,udx_MX,udw_MX


% CasADi setup
opti_nominal = casadi.Opti(); % Create opti instance

% Variables - bounds and initial guess
% States (at mesh and collocation points)
N = 0;
a = opti_nominal.variable(2,N+1);
a_base = opti_nominal.variable(2,1);

opti_nominal.subject_to(1e-3 < a_base < 1);

qA = opti_nominal.variable(auxdata.Ndof,N+1);
qAdot = opti_nominal.variable(auxdata.Ndof,N+1);

% Controls (at mesh points only - piecewise constant over mesh interval)
u = opti_nominal.variable(nStates,1);

K = opti_nominal.variable(4,2);

if VESTIBULARLOSS == 1
    opti_nominal.subject_to(K(3,1) == 0);
    opti_nominal.subject_to(K(3,2) == 0);
    opti_nominal.subject_to(K(4,1) == 0);
    opti_nominal.subject_to(K(4,2) == 0);
else
    opti_nominal.subject_to(K(1,:) > 0);
    opti_nominal.subject_to(K(2,:) < 0);
    opti_nominal.subject_to(K(3,:) > 0);
    opti_nominal.subject_to(K(4,:) < 0);
end



udx = opti_nominal.variable(nStates,nStates);
udw = opti_nominal.variable(nStates,6);

L = opti_nominal.variable(10,N+1);
indices_P = [1 2 3 4;  0 5 6 7; 0 0 8 9 ; 0 0 0 10]';
varIndices_P = [1 5 8 10];
covarIndices_P = [2 3 4 6 7 9];
Lguess = zeros(10,1);
Lguess(varIndices_P) = 1e-3;
if isempty(guess)
    opti_nominal.set_initial(K(1,:),1);
    opti_nominal.set_initial(K(2,:),-1);
    opti_nominal.set_initial(K(3,:),1);
    opti_nominal.set_initial(K(4,:),-1);
    opti_nominal.set_initial(u,0.0);
    opti_nominal.set_initial(a,0);
    opti_nominal.set_initial(udx,1);
    opti_nominal.set_initial(udw,1);
    opti_nominal.set_initial(L,Lguess);
else
    opti_nominal.set_initial(K,guess.Kopt);
    opti_nominal.set_initial(u,guess.u_opt);
    opti_nominal.set_initial(L,guess.L_opt);
    opti_nominal.set_initial(a,guess.a_opt);
    opti_nominal.set_initial(a_base,guess.a_base_opt);
    opti_nominal.set_initial(udx,guess.udx_opt);
    opti_nominal.set_initial(udw,guess.udw_opt);
end


% f_coll_map = f_coll.map(N,'thread',1);
coll_constr = f_coll(a_base,a,qA,qAdot,K,u,0*wM,0*wqPdotdot,0*wPq,0*wPqdot,0*wVq,0*wVqdot,udx,udw);
opti_nominal.subject_to(coll_constr == 0);

% Formulate the NLP
sigmaW = [wM;wqPdotdot;wPq;wPqdot;wVq;wVqdot].*eye(6);

    
Lmat = [L(indices_P(:,1),1) [0;L(indices_P(2:end,2),1)] [0;0;L(indices_P(3:end,3),1)] [0;0;0;L(indices_P(4:end,4),1)]];
Pmat = Lmat*Lmat';

Pdot = udx*Pmat + Pmat*udx' + udw*sigmaW*udw';
Pdot_vec = [Pdot(1:4,1); Pdot(2:4,2); Pdot(3:4,3); Pdot(4,4)];
opti_nominal.subject_to(Pdot_vec == 0);



J_accuracy = L(3)^2 + L(6)^2 + L(8)^2; %  Position error
opti_nominal.subject_to(J_accuracy < (8*pi/180)^2);

J_fb_proprio_state = trace(K(1:2,1:2)*Pmat(3:4,3:4)*K(1:2,1:2)');
J_fb_proprio_accuracy = trace(K(1:2,1:2)*[wPq 0;0 wPqdot]*K(1:2,1:2)');

J_fb_vestibular_state = trace([K(3,1) K(3,2)]*[Pmat(3:4,3:4)]*[K(3,1) K(3,2)]') + ...
                        trace([K(4,1) K(4,2)]*[Pmat(3:4,3:4)]*[K(4,1) K(4,2)]');

J_fb_vestibular_accuracy = trace(K(3:4,1:2)*[wVq 0 ;0 wVqdot]*K(3:4,1:2)');

J_baseline_activation = sumsqr(a_base);

opti_nominal.minimize(Misc.accuracyWeight*J_accuracy + (J_baseline_activation + J_fb_proprio_state + J_fb_proprio_accuracy + J_fb_vestibular_state + J_fb_vestibular_accuracy)); 

opti_nominal.subject_to(a(1:2,1) == 0);
opti_nominal.subject_to(qA(1) == 0);
opti_nominal.subject_to(qAdot(1) == 0);
opti_nominal.subject_to(u(:,1) == 0);
opti_nominal.subject_to(L(varIndices_P,1) > 1e-8);
opti_nominal.set_initial(L(varIndices_P,1) , 1e-1);
opti_nominal.set_initial(L(covarIndices_P,1) , 0);

% Create an NLP solver
output.setup.auxdata = auxdata;
% optionssol.ipopt.nlp_scaling_method = 'none';
optionssol.ipopt.linear_solver = 'ma57';
optionssol.ipopt.tol = 1e-7;
optionssol.ipopt.max_iter = 10000;
% optionssol.ipopt.hessian_approximation = 'limited-memory';
opti_nominal.solver('ipopt',optionssol);
% result = solve_NLPSOL(opti_nominal,optionssol);

% Solve
try

sol = opti_nominal.solve();

    
q_opt = sol.value(qA);
qdot_opt = sol.value(qAdot);
a_opt = sol.value(a);
u_opt = sol.value(u);

% P_opt = sol.value(P);
L_opt = sol.value(L);
J_accuracy_opt = L_opt(3)^2 + L_opt(6)^2 + L_opt(8)^2;
Kopt = sol.value(K);
a_base_opt = sol.value(a_base);
udx_opt = sol.value(udx);
udw_opt = sol.value(udw);

Lmat_opt = [L_opt(indices_P(:,1),1) [0; L_opt(indices_P(2:end,2),1)] [0;0; L_opt(indices_P(3:end,3),1)] [0;0;0;L_opt(indices_P(4:end,4),1)]];
Pmat_opt = Lmat_opt*Lmat_opt';



J_fb_proprio_state_opt = trace(Kopt(1:2,1:2)*Pmat_opt(3:4,3:4)*Kopt(1:2,1:2)');
J_fb_proprio_accuracy_opt = trace(Kopt(1:2,1:2)*[wPq 0;0 wPqdot]*Kopt(1:2,1:2)');

J_fb_vestibular_state_opt =  trace([Kopt(3,1) Kopt(3,2)]*[Pmat_opt(3:4,3:4)]*[Kopt(3,1) Kopt(3,2)]') + ...
                             trace([Kopt(4,1) Kopt(4,2)]*[Pmat_opt(3:4,3:4)]*[Kopt(4,1) Kopt(4,2)]');
J_fb_vestibular_accuracy_opt = trace(Kopt(3:4,1:2)*[wVq 0 ;0 wVqdot]*Kopt(3:4,1:2)');

J_baseline_activation_opt = sum(a_base_opt.^2);


Pmat = Lmat_opt*Lmat_opt';
Pdot_pt1 = udx_opt*Pmat + Pmat*udx_opt';
Pdot_pt2 =  udw_opt*sigmaW*udw_opt';
    

out.J_fb_proprio_state_opt = J_fb_proprio_state_opt;
out.J_fb_vestibular_accuracy_opt = J_fb_vestibular_accuracy_opt;
out.J_fb_proprio_accuracy_opt = J_fb_proprio_accuracy_opt;
out.J_fb_vestibular_state_opt = J_fb_vestibular_state_opt;
out.J_accuracy_opt = J_accuracy_opt;
out.accuracyWeight = Misc.accuracyWeight;


out.a_opt = a_opt;
out.q_opt = q_opt;
out.qdot_opt = qdot_opt;
out.u_opt = u_opt;
out.Pmat_opt = Pmat_opt;
out.L_opt = L_opt;
out.Kopt = Kopt;
out.a_base_opt = a_base_opt;
out.udx_opt = udx_opt;
out.udw_opt = udw_opt;
out.convergence = 1;

catch
    
q_opt = opti_nominal.debug.value(qA);
qdot_opt = opti_nominal.debug.value(qAdot);
a_opt = opti_nominal.debug.value(a);
u_opt = opti_nominal.debug.value(u);

% P_opt = sol.value(P);
L_opt = opti_nominal.debug.value(L);
J_accuracy_opt = L_opt(3)^2 + L_opt(6)^2 + L_opt(8)^2;
Kopt = opti_nominal.debug.value(K);
a_base_opt = opti_nominal.debug.value(a_base);
udx_opt = opti_nominal.debug.value(udx);
udw_opt = opti_nominal.debug.value(udw);

Lmat_opt = [L_opt(indices_P(:,1),1) [0; L_opt(indices_P(2:end,2),1)] [0;0; L_opt(indices_P(3:end,3),1)] [0;0;0;L_opt(indices_P(4:end,4),1)]];
Pmat_opt = Lmat_opt*Lmat_opt';



J_fb_proprio_state_opt = trace(Kopt(1:2,1:2)*Pmat_opt(3:4,3:4)*Kopt(1:2,1:2)');
J_fb_proprio_accuracy_opt = trace(Kopt(1:2,1:2)*[wPq 0;0 wPqdot]*Kopt(1:2,1:2)');

J_fb_vestibular_state_opt =  trace([Kopt(3,1) Kopt(3,2)]*[Pmat_opt(3:4,3:4)]*[Kopt(3,1) Kopt(3,2)]') + ...
                             trace([Kopt(4,1) Kopt(4,2)]*[Pmat_opt(3:4,3:4)]*[Kopt(4,1) Kopt(4,2)]');
J_fb_vestibular_accuracy_opt = trace(Kopt(3:4,1:2)*[wVq 0 ;0 wVqdot]*Kopt(3:4,1:2)');

J_baseline_activation_opt = sumsqr(a_base_opt);


Pmat = Lmat_opt*Lmat_opt';
Pdot_pt1 = udx_opt*Pmat + Pmat*udx_opt';
Pdot_pt2 =  udw_opt*sigmaW*udw_opt';
    

out.J_fb_proprio_state_opt = J_fb_proprio_state_opt;
out.J_fb_vestibular_accuracy_opt = J_fb_vestibular_accuracy_opt;
out.J_fb_proprio_accuracy_opt = J_fb_proprio_accuracy_opt;
out.J_fb_vestibular_state_opt = J_fb_vestibular_state_opt;
out.J_accuracy_opt = J_accuracy_opt;
out.accuracyWeight = Misc.accuracyWeight;


out.a_opt = a_opt;
out.q_opt = q_opt;
out.qdot_opt = qdot_opt;
out.u_opt = u_opt;
out.Pmat_opt = Pmat_opt;
out.L_opt = L_opt;
out.Kopt = Kopt;
out.a_base_opt = a_base_opt;
out.udx_opt = udx_opt;
out.udw_opt = udw_opt;
out.convergence = 0;
end
% 
end