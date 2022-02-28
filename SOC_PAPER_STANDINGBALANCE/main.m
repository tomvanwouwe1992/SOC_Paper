clear all;close all;

TRANSLATION_BOOL = 1;
VESTIBULARLOSS = 0;
SRS = 1;
useNLPsol = 0;
use_solution_as_guess = 0;

% To mimic the uncertainty distribution from Peterka 2002 for the
% 'added' proprioceptive noise by the platform sway
wPq_Platform =     (pi/180*[1e-3 0.125 0.25   0.5   1  2  4]).^2; % Represents angular displacement of the platform (not used for translations)
wPqdot_Platform =  (pi/180*[1e-3 0.3 0.6 1.2 2.4 4.8  9.6]).^2;   % Represents angular velocity of the platform (not used for translations)

if TRANSLATION_BOOL == 1
    wPqdotdot_Platform = (pi/180*[1e-3 1 2 4 8 16 32]).^2; % Represents linear acceleration
else
    wPqdotdot_Platform = (pi/180*[1e-3 1 2 2 2 2  2]).^2;  % Represents angular acceleration
end


saveName = 'solution';
if SRS == 1
    saveName = [saveName '_SRS'];
else
end

if VESTIBULARLOSS == 1
    saveName = [saveName '_VESTIBULARLOSS'];
else
end

saveName = [saveName '_SWAYCTR'];

if TRANSLATION_BOOL == 1
    saveName = [saveName '_TRANSLATION'];
else
    saveName = [saveName '_ROTATION'];
end


colorcode = linspecer(7);
for wP_index = 0:2:0 %0:2:6 % Scale amount of baseline proprioceptive noise
    for wV_index = 0:2:0 %0:2:4 % Scale amount of baseline vestibular noise
        for wM_index = 0:2:0 %0:2:4 % Scale amount of baseline motor noise (muscle activation level
            for wPerturbation_index = 1:7 % Magnitude of rotational perturbationsè
                
                Misc.accuracyWeight = 10; % Small weight on sway minimization ~ accuracy
                
                % Motor noise
                wM_multiplier = 2^wM_index;
                wM = (wM_multiplier*0.01)^2;
                
                % Proprioceptive noise
                wP_multiplier = 2^wP_index;
                wPq_Proprio = (wP_multiplier*0.002)^2;
                wPqdot_Proprio = (wP_multiplier*0.004)^2;
                
                % Vestibular noise
                wV_multiplier = 2^wV_index;
                wVq = (wV_multiplier*0.005)^2;
                wVqdot = (wV_multiplier*0.010)^2;
                
                % Platform perturbation is modelled as additive Gaussian noise
                wPq = wPq_Platform(wPerturbation_index);
                wPqdot = wPqdot_Platform(wPerturbation_index);
                wqPdotdot = wPqdotdot_Platform(wPerturbation_index) + 1e-6;

                guess = [];
                if TRANSLATION_BOOL == 1
                    if VESTIBULARLOSS == 1
                        
                        if use_solution_as_guess == 1
                            load(saveName);
                            guess = solutions(wPerturbation_index).out;
                        else
                        end
                        clear solutions
                        
                        out = OCP_uprightStanding_Translation(Misc,wM,wqPdotdot,wPq_Proprio,wPqdot_Proprio,wVq,wVqdot,VESTIBULARLOSS,guess,SRS);
                        
                    else
                        
                        if use_solution_as_guess == 1
                            load(saveName);
                            guess = solutions(wPerturbation_index).out;
                        else
                        end
                        
                        clear solutions
                        
                        out = OCP_uprightStanding_Translation(Misc,wM,wqPdotdot,wPq_Proprio,wPqdot_Proprio,wVq,wVqdot,VESTIBULARLOSS,guess,SRS);
                        
                    end
                else
                    if VESTIBULARLOSS == 1
                        
                        if use_solution_as_guess == 1
                            load(saveName);
                            guess = solutions(wPerturbation_index).out;
                        else
                        end
                        
                        clear solutions
                        
                        out = OCP_uprightStanding_Rotation(Misc,wM,wqPdotdot,wPq_Proprio,wPqdot_Proprio,wVq,wVqdot,wPq,wPqdot,VESTIBULARLOSS,guess,SRS);
                        
                    else
                        
                        if use_solution_as_guess == 1
                            load(saveName);
                            guess = solutions(wPerturbation_index).out;
                        else
                        end
                        
                        clear solutions
                        
                        out = OCP_uprightStanding_Rotation(Misc,wM,wqPdotdot,wPq_Proprio,wPqdot_Proprio,wVq,wVqdot,wPq,wPqdot,VESTIBULARLOSS,guess,SRS);
                        
                    end
                end
                out.wM_std = sqrt(wM);
                out.wPq_Proprio_std = sqrt(wPq_Proprio);
                out.wPqdot_Proprio_std = sqrt(wPqdot_Proprio);
                out.wVq_std = sqrt(wVq);
                out.wVqdot_std = sqrt(wVqdot);
                
                out.wPq_Platform_std = sqrt(wPq);
                out.wPqdot_Platform_std = sqrt(wPqdot);
                out.wqPdotdot_Platform_std = sqrt(wqPdotdot);

                if exist(saveName,'file') == 2
                    load(saveName);
                else
                    % File does not exist.
                end
                solutions(wPerturbation_index).out = out;
                save(saveName,'solutions');
            end
        end
    end
end

