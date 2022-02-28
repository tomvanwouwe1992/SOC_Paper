clear all; close all;
listing = dir();


h = figure('Name','Stochastic exact forward simulations');
hTabGroup = uitabgroup;

wM_std_VEC = [0.05 ]; % 0.01 0.025 0.05 0.075 0.1
wPq_std_VEC = [3e-4 ];% 2.4e-3];
wPqdot_std_VEC = [2.4e-3 ];% 9.6e-3];

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
            titles = {'BAR unperturbed','CIRCLE unperturbed','OBSTACLE unperturbed','BAR perturbed','CIRCLE perturbed','OBSTACLE perturbed','CIRCLE force field','CIRCLE force field perturbed'};
            
            name = ['result_time_0.8_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
%                         name = ['result_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];

            load(name);
            subplot(2,4,1)
            if isfield(result,'X_stoch_unperturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_unperturbed(1,:,k),result.EE_stoch_unperturbed(2,:,k),'Color',cs(2,:)); hold on
                    title(titles(1));
                end
                % 'Exact' end point error ellipse
                EEendpoint = [squeeze(result.EE_stoch_unperturbed(1,end,:)) squeeze(result.EE_stoch_unperturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95)
                for k = 1:10:length(result.time)
                    % Predicted end point error ellipse
                    C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
                        result.P_EEPos(2,k) result.P_EEPos(3,k)];
                    mu = result.EE_ref(1:2,k);
                    error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            subplot(2,4,5)
            if isfield(result,'X_stoch_perturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(2,:)); hold on
                    title(titles(4));
                end
                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
            end
            axis equal
            
            xlim([-0.1 0.3])
            ylim([0.25 0.55])
            box off
%             name = ['result_CIRCLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
                        name = ['result_time_0.8_CIRCLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];

            load(name)
            subplot(2,4,2)
            if isfield(result,'X_stoch_unperturbed')
                for k=1:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_unperturbed(1,:,k),result.EE_stoch_unperturbed(2,:,k),'Color',cs(1,:)); hold on
                    title(titles(2));
                end
                EEendpoint = [squeeze(result.EE_stoch_unperturbed(1,end,:)) squeeze(result.EE_stoch_unperturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
                for k = 1:10:length(result.time)
                    % Predicted end point error ellipse
                    C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
                        result.P_EEPos(2,k) result.P_EEPos(3,k)];
                    mu = result.EE_ref(1:2,k);
                    error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            subplot(2,4,6)
            if isfield(result,'X_stoch_perturbed')
                for k=1:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(1,:)); hold on
                    title(titles(5));
                end
                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
            end
            axis equal
            
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            
            name = ['result_time_0.8_OBSTACLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
%                         name = ['result_OBSTACLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];

            load(name)
            subplot(2,4,3)
            if isfield(result,'X_stoch_unperturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_unperturbed(1,:,k),result.EE_stoch_unperturbed(2,:,k),'Color',cs(3,:)); hold on
                    title(titles(3));
                end
                EEendpoint = [squeeze(result.EE_stoch_unperturbed(1,end,:)) squeeze(result.EE_stoch_unperturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
                for k = 1:10:length(result.time)
                    % Predicted end point error ellipse
                    C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
                        result.P_EEPos(2,k) result.P_EEPos(3,k)];
                    mu = result.EE_ref(1:2,k);
                    error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            subplot(2,4,7)
            if isfield(result,'X_stoch_perturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(3,:)); hold on
                    title(titles(6));
                end
                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
            end
            axis equal
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            
            name = ['result_time_0.8_CIRCLE_forceField_200_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
%                         name = ['result_CIRCLE_forceField_200_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];

            load(name);
            subplot(2,4,4);
            if isfield(result,'X_stoch_unperturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_unperturbed(1,:,k),result.EE_stoch_unperturbed(2,:,k),'Color',cs(3,:)); hold on
                    title(titles(7));
                end
                EEendpoint = [squeeze(result.EE_stoch_unperturbed(1,end,:)) squeeze(result.EE_stoch_unperturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
                error_ellipse(C,mu,0.95);
                for k = 1:10:length(result.time)
                    % Predicted end point error ellipse
                    C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
                        result.P_EEPos(2,k) result.P_EEPos(3,k)];
                    mu = result.EE_ref(1:2,k);
                    error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            subplot(2,4,8)
            if isfield(result,'X_stoch_perturbed')
                for k=3:9:size(result.EE_stoch_unperturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(3,:)); hold on
                    title(titles(8));
                end
            end
            axis equal
            xlim([-0.1 0.1])
            ylim([0.25 0.55])
            box off
            mtit('first sims')
            
        end
    end
end
%     end
% end


