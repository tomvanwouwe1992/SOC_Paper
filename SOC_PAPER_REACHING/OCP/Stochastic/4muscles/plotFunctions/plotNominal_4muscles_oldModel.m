clear all; close all; clc;
resultList = {'result_time_0.8_CIRCLE_forceField_0_0.05_0.0003_0.0024_OldModel.mat', ...
    'result_time_0.8_BAR_forceField_0_0.05_0.0003_0.0024_OldModel.mat', ...
    'result_time_0.8_OBSTACLE_forceField_0_0.05_0.0003_0.0024_OldModel.mat'};
cs = linspecer(3);
    
titles = {'brachialis','lateral triceps','anterior deltoid','posterior deltoid'};

xlim_1 = [-0.15 0.15]; ylim_1 = [0.25 0.55];
xlim_2 = [0 0.8]; ylim_2 = [0 0.7];
xlim_3 = [0 0.8]; ylim_3 = [0 0.1];

for j = 1:length(resultList)
    load(resultList{j});
    
    subplot(2,6,j)
    plot(result.EEPos(:,1),result.EEPos(:,2),'Color',cs(j,:),'LineWidth',2); hold on;
    for i = 1:10:81
        C = [result.P_EEPos(1,i) result.P_EEPos(2,i); result.P_EEPos(2,i) result.P_EEPos(3,i)];
        mu = result.EEPos(i,:)';
        error_ellipse(C,mu,0.95);
    end
        axis equal; box off; 

    xlim(xlim_1);
    ylim(ylim_1);
    
    subplot(2,6,j+3)
    EEVel = result.EEVel;
    tangentialVel = sqrt(EEVel(:,1).^2 + EEVel(:,2).^2);
    plot(result.time,tangentialVel,'Color',cs(j,:),'LineWidth',2); hold on;
    xlim(xlim_2);
    ylim(ylim_2);
    box off; 
    
    for i = 1:4
    subplot(2,6,6+i)
    EEVel = result.EEVel;
    
%     plotMeanAndVar(result.time,result.a(:,i),squeeze(sqrt(result.Pmat(i,i,:))),cs(j,:)); hold on;
    plot(result.time,result.a(:,i),'Color',cs(j,:),'LineWidth',1); hold on;
    xlim(xlim_3);
    ylim(ylim_3);
    box off; 
    title(titles{i})
    end


end