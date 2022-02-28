clear all; close all; clc;
resultList = {'result_time_0.8_CIRCLE_forceField_0_0.05_0.0003_0.0024.mat', ...
    'result_time_0.8_CIRCLE_forceField_200_0.05_0.0003_0.0024.mat'};
cs = linspecer(4);
cs = [cs(1,:); cs(4,:)];    
titles = {'brachialis','lateral triceps','anterior deltoid','posterior deltoid','biceps short','triceps long'};


meanCCI_elbow = NaN(2,1);
meanCCI_shoulder = NaN(2,1);
meanCCI_BI = NaN(2,1);
for j = 1:length(resultList)
    load(resultList{j});
    
    subplot(3,1,1)
    a1 = result.a(:,1);
    a2 = result.a(:,2);
    a_max = min([a1 a2],[],2);
    CCI_elbow = result.CCI_ElbowUni;
    meanCCI_elbow(j) = mean(CCI_elbow);
    plot(result.time,CCI_elbow,'Color',cs(j,:),'LineWidth',2); hold on;

    
    subplot(3,1,2)
    a1 = result.a(:,3);
    a2 = result.a(:,4);
    a_max = min([a1 a2],[],2);
    CCI_shoulder = result.CCI_ShoulderUni;
    plot(result.time,CCI_shoulder,'Color',cs(j,:),'LineWidth',2); hold on;
    meanCCI_shoulder(j) = mean(CCI_shoulder);

    subplot(3,1,3)
    a1 = result.a(:,5);
    a2 = result.a(:,6);
    a_max = min([a1 a2],[],2);
    CCI_BI = result.CCI_Bi;
    plot(result.time,CCI_BI,'Color',cs(j,:),'LineWidth',2); hold on;
    meanCCI_BI(j) = mean(CCI_BI);
 
end

figure()
b_Elbow = bar([meanCCI_elbow meanCCI_shoulder meanCCI_BI]');
set(b_Elbow(1),'facecolor',cs(1,:));
set(b_Elbow(2),'facecolor',cs(2,:));
set(b_Elbow(3),'facecolor',cs(3,:));
ylim([0 0.0075])
