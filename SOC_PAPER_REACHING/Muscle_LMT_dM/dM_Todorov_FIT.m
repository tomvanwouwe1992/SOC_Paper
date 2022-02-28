clear all;
close all;
%%% Muscle moment arms and lengts wrt to joint angles from Todorov & Li
%%% paper

theta_shoulder = pi/180*[0 180];
theta_elbow = pi/180*[0 180];

theta_shoulder = interp1([0,1],theta_shoulder,0:0.01:1);
theta_elbow = interp1([0,1],theta_elbow,0:0.01:1);
[theta_shoulder_grid,theta_elbow_grid] = meshgrid(theta_shoulder,theta_elbow);

cs = linspecer(6);

% m1 - Biceps long, Brachialis, Brachioradialis (elbow flexor)
figure(1);
subplot(1,2,2)
a_m1_elbow = 0.03;
b_m1_elbow = -0.011;
c_m1_elbow = 1.9;
a_m1_shoulder = 0;
b_m1_shoulder = 0;
c_m1_shoulder = 0.01;
dM_elbow = a_m1_elbow + b_m1_elbow*cos(c_m1_elbow*theta_elbow); hold on;
plot(theta_elbow*180/pi,dM_elbow,'LineWidth',2,'Color',cs(1,:));

% m2 - Triceps lateral (elbow extensor)
subplot(1,2,2)
a_m2_elbow = -0.019;
b_m2_elbow = 0;
c_m2_elbow = 0.01;
a_m2_shoulder = 0;
b_m2_shoulder = 0;
c_m2_shoulder = 0.01;
dM_elbow = a_m2_elbow + b_m2_elbow*cos(c_m2_elbow*theta_elbow); hold on;
plot(theta_elbow*180/pi,dM_elbow,'LineWidth',2,'Color',cs(2,:));

% m3 - Anterior deltoid (shoulder flexor)
subplot(1,2,1)
a_m3_elbow = 0;
b_m3_elbow = 0;
c_m3_elbow = 0.01;
a_m3_shoulder = 0.04;
b_m3_shoulder = -0.008;
c_m3_shoulder = 1.9;
dM_shoulder = a_m3_shoulder + b_m3_shoulder*cos(c_m3_shoulder*theta_shoulder); hold on;
plot(theta_shoulder*180/pi,dM_shoulder,'LineWidth',2,'Color',cs(3,:));

% m4 - Posterior deltoid (shoulder extensor)
subplot(1,2,1)
a_m4_elbow = 0;
b_m4_elbow = 0;
c_m4_elbow = 0.01;
a_m4_shoulder = -0.042;
b_m4_shoulder = 0;
c_m4_shoulder = 0.01;
dM_shoulder = a_m4_shoulder + b_m4_shoulder*cos(c_m4_shoulder*theta_shoulder); hold on;
plot(theta_shoulder*180/pi,dM_shoulder,'LineWidth',2,'Color',cs(4,:));
title('shoulder')
ylabel('dM [m]')

% m5 - Biceps short (shoulder and elbow flexor)
a_m5_elbow = 0.032;
b_m5_elbow = -0.010;
c_m5_elbow = 1.9;
a_m5_shoulder = 0.03;
b_m5_shoulder = -0.011;
c_m5_shoulder = 1.9;
subplot(1,2,1)
dM_shoulder = a_m5_shoulder + b_m5_shoulder*cos(c_m5_shoulder*theta_shoulder); hold on;
plot(theta_shoulder*180/pi,dM_shoulder,'LineWidth',2,'Color',cs(5,:));
subplot(1,2,2)
dM_elbow = a_m5_elbow + b_m5_elbow*cos(c_m5_elbow*theta_elbow); hold on;
plot(theta_elbow*180/pi,dM_elbow,'LineWidth',2,'Color',cs(5,:));
title('elbow')
ylabel('dM [m]')

% m5 - Triceps long (shoulder and elbow extensor)
a_m6_elbow = -0.022;
b_m6_elbow = 0;
c_m6_elbow = 0.01;
a_m6_shoulder = -0.039;
b_m6_shoulder = 0;
c_m6_shoulder = 0.01;
subplot(1,2,1)
dM_shoulder = a_m6_shoulder + b_m6_shoulder*cos(c_m6_shoulder*theta_shoulder); hold on;
plot(theta_shoulder*180/pi,dM_shoulder,'LineWidth',2,'Color',cs(6,:));
ylim([-0.05 0.05])
xlim([0 180])
legend('anterior deltoid','posterior deltoid','short biceps','long triceps')

subplot(1,2,2)
dM_elbow = a_m6_elbow + b_m6_elbow*cos(c_m6_elbow*theta_elbow); hold on;
ylim([-0.05 0.05])
xlim([0 180])

plot(theta_elbow*180/pi,dM_elbow,'LineWidth',2,'Color',cs(6,:));

legend('brachialis','lateral triceps','short biceps','long triceps')

dM_coefficients = [ a_m1_shoulder b_m1_shoulder c_m1_shoulder a_m1_elbow b_m1_elbow c_m1_elbow; ...
                    a_m2_shoulder b_m2_shoulder c_m2_shoulder a_m2_elbow b_m2_elbow c_m2_elbow; ...
                    a_m3_shoulder b_m3_shoulder c_m3_shoulder a_m3_elbow b_m3_elbow c_m3_elbow; ...
                    a_m4_shoulder b_m4_shoulder c_m4_shoulder a_m4_elbow b_m4_elbow c_m4_elbow; ...
                    a_m5_shoulder b_m5_shoulder c_m5_shoulder a_m5_elbow b_m5_elbow c_m5_elbow; ...
                    a_m6_shoulder b_m6_shoulder c_m6_shoulder a_m6_elbow b_m6_elbow c_m6_elbow];
save('dM_coefficients.mat','dM_coefficients');

