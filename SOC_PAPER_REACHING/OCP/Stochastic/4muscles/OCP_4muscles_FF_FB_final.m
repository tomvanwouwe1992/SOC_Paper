function [result,succes] = OCP_4muscles_FF_FB_final(target,forceField,wM_std,wPq_std,wPqdot_std,guessName)
%% SECTION TITLE
% DESCRIPTIVE TEXT
if strcmp(target,'CIRCLE')
    targetNR = 1;
    %     guessName = 'result_CIRCLE.mat';
elseif strcmp(target,'BAR')
    targetNR = 2;
    %     guessName = 'result_BAR.mat';
elseif strcmp(target,'OBSTACLE')
    targetNR = 3;
    %     guessName = 'result_OBSTACLE.mat';
else
    error('Unknown target specified')
end

import casadi.*

% Set-up structure with data that specifies model
auxdata = initializeModelParameters_4muscles();

% Additional simulation settings
T = auxdata.T;
dt = 0.01; auxdata.dt = dt; % time step
N = round(T/dt); auxdata.N = N;
time = 0:dt:T; auxdata.time = time;
nStates = 8; auxdata.nStates = nStates; % #states (6 muscle states + 2 positions + 2 velocities)
wM = (wM_std*ones(2,1)).^2/dt; auxdata.wM = wM; % Motor noise: go from std of continuous noise source to variance of discrete sample
wPq = (wPq_std*ones(2,1)).^2/dt; auxdata.wPq = wPq; % Sensor position noise: go from std of continuous noise source to variance of discrete sample
wPqdot = (wPqdot_std*ones(2,1)).^2/dt; auxdata.wPqdot = wPqdot; % Sensor velocity noise: go from std of continuous noise source to variance of discrete sample

sigma_w = [wM; wPq; wPqdot].*eye(6); auxdata.sigma_w = sigma_w; % Collect noise in covariance matrix

auxdata.forceField = forceField;

%%%% Define CasADi functions - for reasons of efficiency and to compute sensitivities (jacobians) of functions
functions = generateFunctions_OCP_4muscles_FF_FB(auxdata);


opti = casadi.Opti(); % Create opti instance

% Initial and final position of the reaching movement
fsolve_options = optimoptions('fsolve','FiniteDifferenceType','central','StepTolerance',1e-10,'OptimalityTolerance',1e-10);
shoulder_pos_init = 20*pi/180;
shoulder_pos_final = 55*pi/180;
f = @(x)get_px(x,auxdata,shoulder_pos_init);
initial_pos = fsolve(f,ones,fsolve_options);
initial_pos = [shoulder_pos_init; initial_pos];

f = @(x)get_px(x,auxdata,shoulder_pos_final);
final_pos = fsolve(f,ones,fsolve_options);
final_pos = [shoulder_pos_final; final_pos];
EE_final = EndEffectorPos(final_pos,auxdata);


% % Initial guess controls
% guessName = 'result_time_0.8_BAR_forceField_0_0.05_0.0003_0.0024_.mat';
if isempty(guessName)
    X_init = opti.variable(nStates,1); opti.set_initial(X_init, [zeros(4,1); initial_pos; 0;0]);
    X_guess = zeros(8,N+1); X_guess(1:4,:) = 0.01;   X_guess(5,:) = interp1([0 T], [initial_pos(1) final_pos(1)],time); X_guess(6,:) = interp1([0 T], [initial_pos(2) final_pos(2)],time);
    X = opti.variable(nStates,N+1); opti.set_initial(X, X_guess);
    e_ff = opti.variable(4,N+1); opti.set_initial(e_ff, 0.01);
    K = opti.variable(4*4,N+1); opti.set_initial(K, 0.01);
    EE_ref = opti.variable(4,N+1); opti.set_initial(EE_ref, 0.01);
    M = opti.variable(nStates,nStates*N); opti.set_initial(M, 0.01);
    Pmat_init = [1e-6;1e-6;1e-6;1e-6;1e-4;1e-4;1e-7;1e-7;].*eye(8);
    Pmat_i = Pmat_init;
    
    
else
    load(guessName);
    
    % Interpolate guess to new mesh or new timing (if reaching movement timing
    % has changed)
    timeStepsGuess_new = size(result.X,2)-1;
    timeVec_new = 0:T/timeStepsGuess_new:T;
    e_ff_guess = interp1(timeVec_new,result.e_ff,time);
    K_guess = interp1(timeVec_new,result.K',time)';
    X_init_guess = result.X(:,1);
    Pmat_init = [1e-6;1e-6;1e-6;1e-6;1e-4;1e-4;1e-7;1e-7;].*eye(8);
    Pmat_i = Pmat_init;
    [X_guess, M_guess, EE_ref_guess, Pmat_guess] = approximateForwardSim_4muscles(X_init_guess,Pmat_init,e_ff_guess',K_guess,auxdata,functions);
    if abs(EE_ref_guess(1,end)) > 0.1 % control of initial guess is not very stable/precise, better to violate the dynamics and provide trajectories that satisfy the constraints
        X_init = opti.variable(nStates,1); opti.set_initial(X_init, X_init_guess);
        X = opti.variable(nStates,N+1); opti.set_initial(X, interp1(timeVec_new,result.X',time)');
        e_ff = opti.variable(4,N+1); opti.set_initial(e_ff, e_ff_guess');
        K = opti.variable(4*4,N+1); opti.set_initial(K, K_guess);
        EE_ref = opti.variable(4,N+1); opti.set_initial(EE_ref, result.EE_ref);
        M = opti.variable(nStates,nStates*N); opti.set_initial(M, M_guess);
    else
        X_init = opti.variable(nStates,1); opti.set_initial(X_init, X_init_guess);
        X = opti.variable(nStates,N+1); opti.set_initial(X, X_guess);
        e_ff = opti.variable(4,N+1); opti.set_initial(e_ff, e_ff_guess');
        K = opti.variable(4*4,N+1); opti.set_initial(K, K_guess);
        EE_ref = opti.variable(4,N+1); opti.set_initial(EE_ref, EE_ref_guess);
        M = opti.variable(nStates,nStates*N); opti.set_initial(M, M_guess);
    end
end




J_fb = 0;
for i = 1:N
    X_i = X(:,i);
    X_i_plus = X(:,i+1);
    e_ff_i = e_ff(:,i);
    e_ff_i_plus = e_ff(:,i+1);
    
    dX_i = functions.f_forwardMusculoskeletalDynamics(X_i,e_ff_i,0,0,0*wM,0*wPq,0*wPqdot);
    dX_i_plus = functions.f_forwardMusculoskeletalDynamics(X_i_plus,e_ff_i_plus,0,0,0*wM,0*wPq,0*wPqdot);
    opti.subject_to((functions.f_G_Trapezoidal(X_i,X_i_plus,dX_i,dX_i_plus))*1e3 == 0);
    
    M_i = M(:,(i-1)*nStates + 1:i*nStates);
    EE_ref_i = EE_ref(:,i);
    EE_ref_i_plus = EE_ref(:,i+1);
    
    K_i = reshape(K(:,i),4,4);
    K_i_plus = reshape(K(:,i+1),4,4);
    
    DdX_DX_i = functions.f_DdX_DX(X_i,e_ff_i,K_i,EE_ref_i,wM,wPq,wPqdot);
    DdZ_DX_i = functions.f_DdX_DX(X_i_plus,e_ff_i_plus,K_i_plus,EE_ref_i_plus,wM,wPq,wPqdot);
    DdX_Dw_i = functions.f_DdX_Dw(X_i,e_ff_i,K_i,EE_ref_i,wM,wPq,wPqdot);
    
    DG_DX_i = functions.f_DG_DX(DdX_DX_i);
    DG_DZ_i = functions.f_DG_DZ(DdZ_DX_i);
    DG_DW_i = functions.f_DG_DW(DdX_Dw_i);
    
    opti.subject_to(M_i*DG_DZ_i - eye(nStates) == 0);
    J_fb = J_fb + (functions.f_expectedEffort_fb(X_i,Pmat_i,K_i,EE_ref_i,wPq,wPqdot) + trace(Pmat_i(1:4,1:4)))/2;% + trace(Pmat_i(1:6,1:6)); %expectedEffort_fb(i);
    
    % Obstacle
    if targetNR == 3
        if i*dt > T*5/8
            P_q_i = Pmat_i(5:6,5:6);
            P_EEPos_i = functions.f_P_EEPos(X(5:6,i),P_q_i);
            opti.subject_to((P_EEPos_i(1,1) -  0.004^2) < 0);
            opti.subject_to(-1e-4 < EE_ref_i(1) < 1e-4)
        end
    end
    
    Pmat_i = M_i*(DG_DX_i*Pmat_i*DG_DX_i' + DG_DW_i*sigma_w*DG_DW_i')*M_i'; % + dGdW*sigmaW*dGdW'
    
end
J_fb = J_fb + (functions.f_expectedEffort_fb(X_i_plus,Pmat_i,K_i_plus,EE_ref_i_plus,wPq,wPqdot) + trace(Pmat_i(1:4,1:4)))/2;% + trace(Pmat_i(1:6,1:6));

%% Boundary conditions
% Impose mean end effector trajectory to be equal to reference end effect trajectory
EE = [EndEffectorPos(X(5:6,:),auxdata); EndEffectorVel(X(5:6,:),X(7:8,:),auxdata)];
opti.subject_to(EE - EE_ref == 0);

% Initial conditions
opti.subject_to(X(:,1) == X_init); % Set X_init to be equal to the first state
opti.subject_to(X_init(5:8) - [initial_pos;0;0] == 0); % Initial position and velocity
dX_init = functions.f_forwardMusculoskeletalDynamics(X_init,e_ff(:,1),0,0,0*wM,0*wPq,0*wPqdot); % Initial state derivative
opti.subject_to(dX_init(7:8) == 0); % Initial acceleration equals zero (activations need to fullfill requirement)

% Reaching motion must end in the final reach position with zero angular joint velocity
opti.subject_to(functions.f_EEPos(X(5:6,end)) - EE_final == 0);
opti.subject_to(X(7:8,end) == [0;0]);

% Final acceleration equals zero (activations balanced)
dX_end = functions.f_forwardMusculoskeletalDynamics(X(:,end),e_ff(:,end),0,0,0*wM,0*wPq,0*wPqdot); % Initial state derivative
opti.subject_to(dX_end(7:8) == 0); % Initial acceleration equals zero (activations need to fullfill requirement)

% End effector endpoint accuracy
% Constrain the end point position and velocity standard deviation in the x and y
% directions to be below 0.4cm and 2cm/s respectively (depending on the
% target shape)
P_q_final = Pmat_i(5:6,5:6);
P_EEPos_final = functions.f_P_EEPos(X(5:6,end),P_q_final);
P_q_qdot_final = Pmat_i(5:8,5:8);
P_EEVel_final = functions.f_P_EEVel(X(5:6,end),X(7:8,end),P_q_qdot_final);
if targetNR == 1 || targetNR == 3
    opti.subject_to((P_EEPos_final(1,1) - 0.004^2) < 0);
    
else
    %     opti.subject_to(P_EEPos_final(1,1) < 0.025^2);
end
opti.subject_to((P_EEPos_final(2,2) - 0.004^2) < 0);
opti.subject_to((P_EEVel_final(1,1) - 0.05^2) < 0);
opti.subject_to((P_EEVel_final(2,2) - 0.05^2) < 0);

% Limit variance on activations in endpoint
% opti.subject_to((Pmat_i(1,1) - 0.01^2) < 0);opti.subject_to((Pmat_i(2,2) - 0.01^2) < 0);opti.subject_to((Pmat_i(3,3) - 0.01^2) < 0);
% opti.subject_to((Pmat_i(4,4) - 0.01^2) < 0);


% Bounds on the feedforward excitations, activations and joint angles
opti.subject_to(0.001 < e_ff(:) < 1);
opti.subject_to(0.001 < X(1:4,:) < 1);
opti.subject_to(0 < X(5,:) < 180);
opti.subject_to(0 < X(6,:) < 180);

%% Cost function
opti.minimize(1e3*((sumsqr(e_ff)+sumsqr(X(1:4,:)))/2+J_fb)*dt);

%% Setup solver
% optionssol.ipopt.nlp_scaling_method = 'gradient-based';
optionssol.ipopt.linear_solver = 'ma57';
optionssol.ipopt.tol = 1e-3; % output.setup.nlp.ipoptoptions.tolerance;
optionssol.ipopt.dual_inf_tol = 3e-4;
optionssol.ipopt.constr_viol_tol = 1e-8;
optionssol.ipopt.max_iter = 10000;

optionssol.ipopt.hessian_approximation = 'limited-memory';
opti.solver('ipopt',optionssol);

try
    tic;
    % result = solve_NLPSOL(opti,optionssol);
    sol = opti.solve();
    toc
    e_ff_sol = sol.value(e_ff);
    K_sol = sol.value(K);
    X_init_sol = sol.value(X_init);
    [X_sol, ~, EE_ref_sol, Pmat_sol] = approximateForwardSim_4muscles(X_init_sol,Pmat_init,e_ff_sol,K_sol,auxdata,functions);
    
    for i = 1:N+1
        Pmat_sol_i = Pmat_sol(:,:,i);
        P_q_sol_i = Pmat_sol_i(5:6,5:6);
        P_EEPos_mat_i = functions.f_P_EEPos(X_sol(5:6,i),P_q_sol_i);
        P_EEPos_sol(:,i) = full([P_EEPos_mat_i(1,1); P_EEPos_mat_i(2,1); P_EEPos_mat_i(2,2)]);
        P_qdot_sol_i = Pmat_sol_i(5:8,5:8);
        P_EEVel_mat_i = functions.f_P_EEVel(X_sol(5:6,i),X_sol(7:8,i),P_qdot_sol_i);
        P_EEVel_sol(:,i) = full([P_EEVel_mat_i(1,1); P_EEVel_mat_i(2,1); P_EEVel_mat_i(2,2)]);
    end
    
    EEPos_sol = EndEffectorPos(X_sol(5:6,:),auxdata)';
    EEVel_sol = EndEffectorVel(X_sol(5:6,:),X_sol(7:8,:),auxdata)';
    
    clear result;
    result.e_ff = e_ff_sol';
    result.X = X_sol;
    result.a = X_sol(1:4,:)';
    result.q = X_sol(5:6,:)';
    result.qdot = X_sol(7:8,:)';
    result.K = K_sol;
    result.M = sol.value(M);
    result.Pmat = Pmat_sol;
    result.time = 0:dt:T;
    result.auxdata = auxdata;
    result.EEPos = EEPos_sol;
    result.EEVel = EEVel_sol;
    result.P_EEPos = P_EEPos_sol;
    result.P_EEVel = P_EEVel_sol;
    result.EE_ref = EE_ref_sol;
    
    % Cost function
    %-feedback
    J_fb = NaN(auxdata.N+1,1);
    J_fb_corrective = NaN(auxdata.N+1,1);
    J_fb_sensoryNoise = NaN(auxdata.N+1,1);
    
    
    for i = 1:auxdata.N+1
        J_fb(i) = full(functions.f_expectedEffort_fb(X_sol(:,i),Pmat_sol(:,:,i),reshape(K_sol(:,i),4,4),EE_ref_sol(:,i),wPq,wPqdot));
        J_fb_corrective(i) = full(functions.f_expectedEffort_fb_corrective(X_sol(:,i),Pmat_sol(:,:,i),reshape(K_sol(:,i),4,4),EE_ref_sol(:,i)));
        J_fb_sensoryNoise(i) = full(functions.f_expectedEffort_fb_sensoryNoise(reshape(K_sol(:,i),4,4),wPq,wPqdot));
    end
    J_fb_total = sum(J_fb);
    J_ff_total = (sumsqr(e_ff_sol)+sumsqr(X_sol(1:4,:)))/2;
    result.J_fb = J_fb;
    result.J_fb_corrective = J_fb_corrective;
    result.J_fb_sensoryNoise = J_fb_sensoryNoise;
    result.J_fb_total = J_fb_total;
    result.J_ff_total = J_ff_total;
    
    
    
    result.CCI_ElbowUni = computeCocontraction(result.a(:,1),result.a(:,2));
    result.CCI_ShoulderUni = computeCocontraction(result.a(:,3),result.a(:,4));
    %     result.CCI_Bi = computeCocontraction(result.a(:,5),result.a(:,6));
    succes = true;
    
catch
    succes = false;
end



