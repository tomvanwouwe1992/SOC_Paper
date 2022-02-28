clear all; 
close all;

load('dM_coefficients');
load('LMT_coefficients');

% Evaluate at a position where the shoulder is in 45° and elbow in 90°
theta_shoulder = pi/4;
theta_elbow = pi/2;

dtheta_shoulder = 4*pi/180*[-720 720];
dtheta_elbow = 4*pi/180*[-720 720];

dtheta_shoulder = interp1([0,1],dtheta_shoulder,0:0.01:1);
dtheta_elbow = interp1([0,1],dtheta_elbow,0:0.01:1);
[dtheta_shoulder_grid,dtheta_elbow_grid] = meshgrid(dtheta_shoulder,dtheta_elbow);
titles = {'m1 - Brachialis','m2 - Lateral triceps','m3 - anterior deltoid','m4 - posterior deltoid','m5 - biceps short','m6 - triceps long'};
    figure()

for i = 1:6
    subplot(2,3,i)
    VMT = evaluate_VMT(dM_coefficients(i,1),dM_coefficients(i,2),dM_coefficients(i,3),dM_coefficients(i,4),dM_coefficients(i,5),dM_coefficients(i,6),LMT_coefficients(i,2),theta_shoulder,theta_elbow,dtheta_shoulder_grid,dtheta_elbow_grid);
    contour(180/pi*dtheta_shoulder_grid, 180/pi*dtheta_elbow_grid, VMT,'ShowText','on');
    xlabel('shoulder [°/s]')
    ylabel('elbow  [°/s]')
    title(titles(i))
end

