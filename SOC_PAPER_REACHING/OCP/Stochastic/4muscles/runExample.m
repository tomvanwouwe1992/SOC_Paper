% Run some examples

wM_std_VEC = [0.05]; %0.01 0.025 0.05 0.075 0.1
wPq_std_VEC = [3e-4]; % 6e-4];% 1.2e-3];% 2.4e-3];
wPqdot_std_VEC = [2.4e-3 ];%4.8e-3];% 9.6e-3];
addpath("C:\Program Files\casadi-windows-matlabR2016a-v3.5.5")
listing = dir();

wM_std = wM_std_VEC(1);
wPq_std = wPq_std_VEC(1);
wPqdot_std = wPqdot_std_VEC(1);
forceField = 0;

% Example 1
target = 'CIRCLE';
targetGuess = 'OBSTACLE';
saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '.mat'];
[resultShooting_1,succesShooting_1] = OCP_4muscles_FF_FB_final(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);

% Example 2
target = 'BAR';
targetGuess = 'CIRCLE';
saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '.mat'];
[resultShooting_2,succesShooting_2] = OCP_4muscles_FF_FB_final(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);

% Example 3
target = 'OBSTACLE';
targetGuess = 'CIRCLE';
saveName = ['result_time_0.8_' target '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
guessName = ['result_time_0.8_' targetGuess '_forceField_' num2str(forceField) '_' num2str(wM_std) '_' num2str( wPq_std) '_' num2str(wPqdot_std) '.mat'];
[resultShooting_3,succesShooting_3] = OCP_4muscles_FF_FB_final(target,forceField,wM_std,wPq_std,wPqdot_std,guessName);
