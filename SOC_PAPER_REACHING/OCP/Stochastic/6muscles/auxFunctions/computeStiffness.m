function Stiffness = computeStiffness(resultName)
import casadi.*

load(resultName)

% FORWARD KINEMATICS
q_MX = MX.sym('q_MX',2);       % Controls (muscle excitations - feedforward)
EEPos_MX = EndEffectorPos(q_MX,result.auxdata);
JAC_EEPos_MX = jacobian(EEPos_MX,q_MX');
f_JAC_EEPos = Function('f_JAC_EEPos',{q_MX},{JAC_EEPos_MX});
f_EEPos = Function('f_EEPos',{q_MX},{EEPos_MX});

% FORWARD DYNAMICS
e_ff_MX = MX.sym('u_MX',6);       % Controls (muscle excitations - feedforward)
X_MX = MX.sym('X_MX',result.auxdata.nStates);    % States (muscle activations and joint kinematics)
wM_MX = MX.sym('wM_MX',2);        % Motor noise
wPq_MX = MX.sym('wPq_MX',2);      % Sensor position noise
wPqdot_MX = MX.sym('wPqdot_MX',2);% Sensor velocity noise
EE_ref_MX = MX.sym('EE_ref_MX', 4); % Reference end-effector kinematics
EE_MX = [EndEffectorPos(X_MX(7:8),result.auxdata); EndEffectorVel(X_MX(7:8),X_MX(9:10),result.auxdata)]; % End-effector kinematics computed from joint kinematics
K_MX = MX.sym('K_MX',6,4);        % Feedback gains
e_fb_MX = K_MX*((EE_MX - EE_ref_MX) + [wPq_MX;wPqdot_MX]); % Feedback excitations composed from the feedback of the end-effector kinematics error, corrupted by sensor noise
u_MX = e_ff_MX + e_fb_MX;         % Total muscle excitations (ff + fb)
T_EXT_MX = MX.sym('T_EXT_MX',2,1); % External torque
result.auxdata.forceField = 0;
% unperturbed stochastic forward dynamics
dX_MX = forwardMusculoskeletalDynamics_motorNoise(X_MX,u_MX,T_EXT_MX,wM_MX,result.auxdata);
dX_MX = dX_MX(9:10);
f_forwardMusculoskeletalDynamics = Function('f_forwardMusculoskeletalDynamics',{X_MX,e_ff_MX,K_MX,T_EXT_MX,EE_ref_MX,wM_MX,wPq_MX,wPqdot_MX},{dX_MX});



sol_T_EXT_MX = MX.sym('sol_T_EXT_MX',2);
for t = 1:81
    rf = rootfinder('rf','newton',struct('x',sol_T_EXT_MX,'g',f_forwardMusculoskeletalDynamics([result.a(t,:)';result.q(t,:)';result.qdot(t,:)'],result.e_ff(t,:)',reshape(result.K(:,t),6,4),sol_T_EXT_MX,result.EE_ref(:,t),zeros(2,1),zeros(2,1),zeros(2,1))),struct('abstol',1e-16));
    solution = rf(zeros(2,1),[]);
    
    J = full(f_JAC_EEPos(result.q(t,:)')); % xdot = J*qdot
    
    % minus sign because we computed the torque required to have
    % acceleration zero given the produced muscle torques.... So the
    % opposite is the force exerted on the environment.
    F(:,t) = -inv(J')*full(solution); % T = J'*Fx
    [Fa(:,t),Fp(:,t),~,~,~,~,~,~,~] = getMuscleForce(result.q(t,:)',result.qdot(t,:)',result.auxdata);
end

stepAngle = pi/16;
Stiffness = NaN(50,32);
t_i = 0;
for t = 1:5:66
    t_i = t_i + 1;
    theta_i = 0;
    for theta = 0:stepAngle:2*pi-stepAngle
        theta_i = theta_i + 1;
        pert_p = 0.005*[cos(theta);sin(theta)];
        
        % Part not really necessary because feedback comes from the
        % end-effector position...
        J = full(f_JAC_EEPos(result.q(t,:)'));
        delta_q = J^(-1)*pert_p;
        q_perturbed = result.q(t,:)' + delta_q;
        EE_pos_baseline = full(f_EEPos(result.q(t,:)'));
        EE_pos_perturbed = full(f_EEPos(q_perturbed));
        
        % Simulate for the next 200ms how the muscle activations are
        % affected
        a_sim = NaN(6,16);
        e_fb = NaN(6,16);
        
        a_sim(:,1) = result.a(t,:)';
        
        for k = 1:15
            if t+k-1 > 81
                a_sim(:,k+1) = a_sim(:,k);
            else
                e_fb(:,k+1) = reshape(result.K(:,t+k-1),6,4)*([EE_pos_perturbed-EE_pos_baseline; zeros(2,1)]);
                e_ff = result.e_ff(t+k-1,:)';
                da = ( e_fb(:,k+1)+e_ff-a_sim(:,k))/result.auxdata.tau;
                a_sim(:,k+1) = a_sim(:,k) + da*0.01;
            end
        end
        a_diff = result.a';
        a_diff(:,t:t+15) = a_sim - a_diff(:,t:t+15);
        for k = 1:15
            if t+k-1 > 81
                index  = 81;
            else
                index  = t+k-1;
            end
            % Part not really necessary because feedback comes from the
            % end-effector position...
            J = full(f_JAC_EEPos(result.q(index,:)'));
            delta_q = J^(-1)*pert_p;
            q_perturbed = result.q(index,:)' + delta_q;
            
            rf = rootfinder('rf','newton',struct('x',sol_T_EXT_MX,'g',f_forwardMusculoskeletalDynamics([a_sim(:,k);result.q(index,:)' + delta_q;result.qdot(index,:)'],result.e_ff(t,:)',reshape(result.K(:,index),6,4),sol_T_EXT_MX,result.EE_ref(:,index),zeros(2,1),zeros(2,1),zeros(2,1))),struct('abstol',1e-16));
            solution = rf(zeros(2,1),[]);
            
            J = full(f_JAC_EEPos(q_perturbed)); % xdot = J*qdot
            
            % minus sign because we computed the torque required to have
            % acceleration zero given the produced muscle torques.... So the
            % opposite is the force exerted on the environment.
            T_perturbed(:,k) = -full(solution);
            F_perturbed(:,k) = -(J')^(-1)*full(solution); % T = J'*Fx
            F_restore(:,k) = F_perturbed(:,k)-F(:,index);
            F_restore_norm(:,k) = pert_p'*F_restore(:,k)/(norm(pert_p));
            Stiffness_trial(:,k) = F_restore_norm(:,k)/norm(pert_p);
            [Fa_perturbed(:,k),Fp_perturbed(:,k),~,~,~,~,~,~,~] = getMuscleForce(result.q(index,:)'+ delta_q,result.qdot(index,:)',result.auxdata);
            
        end
        Stiffness(t_i,theta_i) = Stiffness_trial(15);
    end
end
end



