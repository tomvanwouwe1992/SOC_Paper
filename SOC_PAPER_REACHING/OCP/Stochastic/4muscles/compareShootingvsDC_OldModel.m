% This file runs a comparison between the shooting and direct collocation
% implementation of the covariance matrix propagation, and this for a
% specific noise settings and target

% Note that the iterations for the shooting simulation run faster than for
% the DC simulation clearly for the BAR and CIRCLE case. However, for the
% OBSTACLE case the iteration times become more similar. This has to do
% with the increase in the number of path constraints in the OBSTACLE case
% where the sparsity is exploited in the DC implementation.

% The DC problems seem to take longer to converge (especially when the
% problems are initialized with a feasible and relatively good initial guess).
% The computational efficiency of the two implementations could probably be
% more equivalent if for the DC implementation some regularization is
% introduced. However, despite usually with regularization we can get equivalent or very similar results, 
% this would off course slightly change the underlying problem to some degree and make the formulation less clean. 


wM_std_VEC = [0.05]; %0.01 0.025 0.05 0.075 0.1
wPq_std_VEC = [3e-4]; % 6e-4];% 1.2e-3];% 2.4e-3];
wPqdot_std_VEC = [2.4e-3 ];%4.8e-3];% 9.6e-3];
addpath("C:\Program Files\casadi-windows-matlabR2016a-v3.5.5")
listing = dir();
for i = 1:length(wM_std_VEC)
    for j = 1:length(wPq_std_VEC)
        for k = 1:length(wPqdot_std_VEC)
            wM_std = wM_std_VEC(i);
            wPq_std = wPq_std_VEC(j);
            wPqdot_std = wPqdot_std_VEC(k);
            forceField = 0;
            
            % Example 1
            target = 'CIRCLE';
            targetGuess = 'CIRCLE';
            saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            [resultDC_1,succesDC_1] = OCP_4muscles_FF_FB_final_OldModel_DCversion(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);         
            [resultShooting_1,succesShooting_1] = OCP_4muscles_FF_FB_final_OldModel(target,forceField,wM_std,wPq_std,wPqdot_std,guessName); 

            % Example 1
            target = 'BAR';
            targetGuess = 'CIRCLE';
            saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            [resultDC_2,succesDC_2] = OCP_4muscles_FF_FB_final_OldModel_DCversion(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);         
            [resultShooting_2,succesShooting_2] = OCP_4muscles_FF_FB_final_OldModel(target,forceField,wM_std,wPq_std,wPqdot_std,guessName); 

            % Example 1
            target = 'OBSTACLE';
            targetGuess = 'CIRCLE';
            saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '_OldModel.mat'];
            [resultDC_3,succesDC_3] = OCP_4muscles_FF_FB_final_OldModel_DCversion(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);         
            [resultShooting_3,succesShooting_3] = OCP_4muscles_FF_FB_final_OldModel(target,forceField,wM_std,wPq_std,wPqdot_std,guessName); 
                  

        end
    end
end