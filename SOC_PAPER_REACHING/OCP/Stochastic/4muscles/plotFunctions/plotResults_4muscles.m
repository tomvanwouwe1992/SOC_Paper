function [] = plotResults_4muscles(result)

h = figure('Name','Optimal states, controls and other things');
hTabGroup = uitabgroup;

t_end = result.time(end);


tab = uitab(hTabGroup, 'Title', ['Exc/act']);
axes('parent',tab);
titles = {'m1 - Brachialis','m2 - Lateral triceps','m3 - anterior deltoid','m4 - posterior deltoid'};
ymax = max(max(result.e_ff));
for i = 1:4
    subplot(3,2,i)
    stairs(result.time,result.e_ff(:,i)); hold on;
    stdTraj = sqrt(squeeze(result.Pmat(i,i,:)));
    plotMeanAndVar(result.time,result.a(:,i),stdTraj,'b');
    title(titles(i))
    ylim([-0.1 0.1]);
    xlim([0 t_end]);
    xlabel('time [s]');
    ylabel('exc/act [-]');
end

tab = uitab(hTabGroup, 'Title', ['Kinematics']);
axes('parent',tab);
titles = {'shoulder','elbow'};
for i = 1:2
    subplot(2,2,i)
    stdTraj = sqrt(squeeze(result.Pmat(i+4,i+4,:)));
    plotMeanAndVar(result.time,180/pi*result.q(:,i),180/pi*stdTraj,'b'); 
    title(titles(i))
    ylim([0 180]);
    xlim([0 t_end]);
    xlabel('time [s]');
    ylabel('angle [°]');
end

titles = {'shoulder','elbow'};
for i = 1:2
    subplot(2,2,i+2)
        stdTraj = sqrt(squeeze(result.Pmat(i+6,i+6,:)));

    plotMeanAndVar(result.time,180/pi*result.qdot(:,i),180/pi*stdTraj,'b'); 
    title(titles(i))
    ylim([-360 360]);
    xlim([0 t_end]);
    xlabel('time [s]');
    ylabel('angular velocity [°/s]');
end


tab = uitab(hTabGroup, 'Title', 'normalized l_M');
axes('parent',tab);
titles = {'m1 - Brachialis','m2 - Lateral triceps','m3 - anterior deltoid','m4 - posterior deltoid','m5 - biceps short','m6 - triceps long'};
% LMT_vector = evaluate_LMT_vector(result.auxdata.dM_coefficients,result.auxdata.LMT_coefficients,result.q(:,1)',result.q(:,2)')';
% 
% for i = 1:4
%     subplot(3,2,i)
%     % Compute muscle length
%     plot(result.time,LMT_vector(:,i),'LineWidth',2);
%     title(titles(i))
%     ylim([0.4 1.6]);
%     xlim([0 t_end]);
%     xlabel('time [s]');
%     ylabel('l_M');
% end


tab = uitab(hTabGroup, 'Title', 'end-effector trajectory');
axes('parent',tab);
subplot(1,3,1)
stdTraj = sqrt(result.P_EEPos(1,:)');
plotMeanAndVar(result.time,result.EEPos(:,1),stdTraj,'b');
title('x-pos')
ylim([-0.1 0.1]);
xlim([0 t_end]);
xlabel('time [s]');
ylabel('x-pos');
subplot(1,3,2)
stdTraj = sqrt(result.P_EEPos(3,:)');
plotMeanAndVar(result.time,result.EEPos(:,2),stdTraj,'b');title('y-pos')
ylim([0 1]);
xlim([0 t_end]);
xlabel('time [s]');
ylabel('y-pos');

subplot(1,3,3)
plot(result.EEPos(:,1),result.EEPos(:,2),'LineWidth',2); hold on;
for i = 1:10:length(result.time)
Cov = [result.P_EEPos(1,i) result.P_EEPos(2,i); result.P_EEPos(2,i) result.P_EEPos(3,i)];
error_ellipse(Cov,[result.EEPos(i,1);result.EEPos(i,2)],0.95);
end
title('2D traj')
ylim([0 1]);
xlim([-0.1 0.1]);
xlabel('x-pos');
ylabel('y-pos');
axis equal


tab = uitab(hTabGroup, 'Title', ['CCI']);
axes('parent',tab);
titles = {'Brachialis - Lateral triceps','anterior deltoid - posterior deltoid','biceps short - triceps long'};

subplot(3,1,1)
plot(result.time,result.CCI_ElbowUni,'LineWidth',2);
title(titles(1))
ylim([0 1]);
xlim([0 t_end]);
xlabel('time [s]');
ylabel('CCI [-]');

subplot(3,1,2)
plot(result.time,result.CCI_ShoulderUni,'LineWidth',2);
title(titles(1))
ylim([0 1]);
xlim([0 t_end]);
xlabel('time [s]');
ylabel('CCI [-]');




