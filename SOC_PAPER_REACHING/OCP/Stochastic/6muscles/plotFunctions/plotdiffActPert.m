clear all; close all;
listing = dir();

figure();


h = figure('Name','Stochastic exact forward simulations');
hTabGroup = uitabgroup;

wM_std_VEC = 0.05; %[0.01 0.025 0.05 0.075 0.1];
wPq_std_VEC = 3e-4; %[3e-4 6e-4 1.2e-3];% 2.4e-3];
wPqdot_std_VEC = 4.8e-3; %[2.4e-3 4.8e-3];% 9.6e-3];
titles = {'m1 - Brachialis','m2 - Lateral triceps','m3 - anterior deltoid','m4 - posterior deltoid','m5 - biceps short','m6 - triceps long'};

ctr = 1;
cs = linspecer(3);
for i = 1:length(wM_std_VEC)
    for j = 1:length(wPq_std_VEC)
        for s = 1:length(wPqdot_std_VEC)
            wM_std = wM_std_VEC(i);
            wPq_std = wPq_std_VEC(j);
            wPqdot_std = wPqdot_std_VEC(s);
            
            tab = uitab(hTabGroup, 'Title', [num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std)]);
            axes('parent',tab);
            
            name = ['result_CIRCLE_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            load(name)
            % for j = 1:length(listing)
            %     if contains(listing(j).name,'.mat')
            %         name = listing(j).name;
            %         load(name);
            t = 0:0.01:0.3;
            
            subplot(3,2,1)
            diffAct = squeeze(result.X_stoch_perturbed(1,:,:)) - result.X(1,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            subplot(3,2,2)
            diffAct = squeeze(result.X_stoch_perturbed(2,:,:)) - result.X(2,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            subplot(3,2,3)
            diffAct = squeeze(result.X_stoch_perturbed(3,:,:)) - result.X(3,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            subplot(3,2,4)
            diffAct = squeeze(result.X_stoch_perturbed(4,:,:)) - result.X(4,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            subplot(3,2,5)
            diffAct = squeeze(result.X_stoch_perturbed(5,:,:)) - result.X(5,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            subplot(3,2,6)
            diffAct = squeeze(result.X_stoch_perturbed(6,:,:)) - result.X(6,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(1,:)); hold on;
            
            name = ['result_BAR_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            load(name)
            % for j = 1:length(listing)
            %     if contains(listing(j).name,'.mat')
            %         name = listing(j).name;
            %         load(name);
            subplot(3,2,1)
            diffAct = squeeze(result.X_stoch_perturbed(1,:,:)) - result.X(1,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            subplot(3,2,2)
            diffAct = squeeze(result.X_stoch_perturbed(2,:,:)) - result.X(2,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            subplot(3,2,3)
            diffAct = squeeze(result.X_stoch_perturbed(3,:,:)) - result.X(3,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            subplot(3,2,4)
            diffAct = squeeze(result.X_stoch_perturbed(4,:,:)) - result.X(4,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            subplot(3,2,5)
            diffAct = squeeze(result.X_stoch_perturbed(5,:,:)) - result.X(5,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            subplot(3,2,6)
            diffAct = squeeze(result.X_stoch_perturbed(6,:,:)) - result.X(6,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(2,:));
            
            name = ['result_OBSTACLE_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            load(name)
            % for j = 1:length(listing)
            %     if contains(listing(j).name,'.mat')
            %         name = listing(j).name;
            %         load(name);
            subplot(3,2,1)
            diffAct = squeeze(result.X_stoch_perturbed(1,:,:)) - result.X(1,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(1));
            box off;
            subplot(3,2,2)
            diffAct = squeeze(result.X_stoch_perturbed(2,:,:)) - result.X(2,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(2));
            box off;
            subplot(3,2,3)
            diffAct = squeeze(result.X_stoch_perturbed(3,:,:)) - result.X(3,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(3));
            box off;
            subplot(3,2,4)
            diffAct = squeeze(result.X_stoch_perturbed(4,:,:)) - result.X(4,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(4));
            box off;
            subplot(3,2,5)
            diffAct = squeeze(result.X_stoch_perturbed(5,:,:)) - result.X(5,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(5));
            box off;
            subplot(3,2,6)
            diffAct = squeeze(result.X_stoch_perturbed(6,:,:)) - result.X(6,:)';
            plot(t, mean(diffAct(8:38,:),2),'LineWidth',2,'Color',cs(3,:));
            ylim([-0.05 0.05]);
            title(titles(6));
            box off;
            
        end
    end
end
%     end
% end


