clear all; close all;
listing = dir();


figure();

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
            
            titles = {'CIRCLE perturbed','BAR perturbed','OBSTACLE perturbed'};
            
            name = ['result_time_0.8_CIRCLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            %                         name = ['result_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            
            load(name);
            
            subplot(2,3,1)
            if isfield(result,'X_stoch_perturbed')
                for k=1:10:size(result.EE_stoch_perturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(1,:)); hold on
                    title(titles(1));
                end
            end
            axis equal
            
            xlim([-0.1 0.1])
            ylim([0.25 0.57])
            box off
            subplot(2,3,4)
            if isfield(result,'X_stoch_perturbed')
                for k=1:1:size(result.EE_stoch_perturbed,3)
                    scatter(result.EE_stoch_perturbed(1,end,k)-result.EE_ref(1,end),result.EE_stoch_perturbed(2,end,k)-result.EE_ref(2,end),'x','MarkerEdgeColor',cs(1,:)); hold on;
                end

                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint)-result.EE_ref(1:2,end)';
                error_ellipse(C,mu,0.95);
            end
            axis equal
            
            xlim([-0.03 0.03])
            ylim([-0.03 0.03])
            box off
            %
            
            
                        name = ['result_time_0.8_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            %                         name = ['result_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat']
            
            load(name);
            
            subplot(2,3,2)
            if isfield(result,'X_stoch_perturbed')
                for k=2:9:size(result.EE_stoch_perturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(2,:)); hold on
                end
                                    title(titles(2));

                % 'Exact' end point error ellipse
                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
%                 error_ellipse(C,mu,0.95)
                for k = 1:10:length(result.time)
%                     % Predicted end point error ellipse
%                     C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
%                         result.P_EEPos(2,k) result.P_EEPos(3,k)];
%                     mu = result.EE_ref(1:2,k);
%                     error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            
            xlim([-0.1 0.25])
            ylim([0.25 0.57])
            box off
            subplot(2,3,5)
            if isfield(result,'X_stoch_perturbed')
                for k=1:1:size(result.EE_stoch_perturbed,3)
                    scatter(result.EE_stoch_perturbed(1,end,k)-result.EE_ref(1,end),result.EE_stoch_perturbed(2,end,k)-result.EE_ref(2,end),'x','MarkerEdgeColor',cs(2,:)); hold on;
                end

                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint)-result.EE_ref(1:2,end)';
                error_ellipse(C,mu,0.95);
            end
            axis equal
            
            xlim([-0.10 0.25])
            ylim([-0.03 0.03])
            box off
            
            
                                    name = ['result_time_0.8_OBSTACLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            %                         name = ['result_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            
            load(name);
            
            subplot(2,3,3)
            if isfield(result,'X_stoch_perturbed')
                for k=2:9:size(result.EE_stoch_perturbed,3)
                    plot(result.EE_stoch_perturbed(1,:,k),result.EE_stoch_perturbed(2,:,k),'Color',cs(3,:)); hold on
                end
                                    title(titles(3));

                % 'Exact' end point error ellipse
                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint);
%                 error_ellipse(C,mu,0.95)
                for k = 1:10:length(result.time)
%                     % Predicted end point error ellipse
%                     C = [result.P_EEPos(1,k) result.P_EEPos(2,k);
%                         result.P_EEPos(2,k) result.P_EEPos(3,k)];
%                     mu = result.EE_ref(1:2,k);
%                     error_ellipse(C,mu,0.95,1,'style','-.');
                end
            end
            axis equal
            
            xlim([-0.1 0.1])
            ylim([0.25 0.57])
            box off
            subplot(2,3,6)
            if isfield(result,'X_stoch_perturbed')
                for k=1:1:size(result.EE_stoch_perturbed,3)
                    scatter(result.EE_stoch_perturbed(1,end,k)-result.EE_ref(1,end),result.EE_stoch_perturbed(2,end,k)-result.EE_ref(2,end),'x','MarkerEdgeColor',cs(3,:)); hold on;
                end

                EEendpoint = [squeeze(result.EE_stoch_perturbed(1,end,:)) squeeze(result.EE_stoch_perturbed(2,end,:))];
                C = cov(EEendpoint);
                mu = mean(EEendpoint)-result.EE_ref(1:2,end)';
                error_ellipse(C,mu,0.95);
            end
            axis equal
            
            xlim([-0.03 0.03])
            ylim([-0.03 0.03])
            box off
            
        end
    end
end


