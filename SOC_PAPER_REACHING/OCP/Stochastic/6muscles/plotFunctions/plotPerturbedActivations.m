clear all; close all;
listing = dir();


figure();

wM_std_VEC = [0.05 ]; % 0.01 0.025 0.05 0.075 0.1
wPq_std_VEC = [3e-4 ];% 2.4e-3];
wPqdot_std_VEC = [2.4e-3 ];% 9.6e-3];
titles = {'brachialis','lateral triceps','anterior deltoid','posterior deltoid','biceps short','triceps long'};

ctr = 1;
cs = linspecer(3);
for d = 1:length(wM_std_VEC)
    for j = 1:length(wPq_std_VEC)
        for s = 1:length(wPqdot_std_VEC)
            wM_std = wM_std_VEC(d);
            wPq_std = wPq_std_VEC(j);
            wPqdot_std = wPqdot_std_VEC(s);
            
            
            name = ['result_time_0.8_CIRCLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            load(name);
            
            for i = 1:6
                subplot(3,2,i)
                if isfield(result,'X_stoch_perturbed_3')
                    spm1d.plot.plot_meanSD((squeeze(result.X_stoch_perturbed_3(i,:,:))-result.X(i,:)')','color',cs(1,:)); hold on
                    
                    title(titles(i));
                end
                box off  
                ylim([-0.06 0.06])
                xlim([17 47])
            end
            
            
            name = ['result_time_0.8_BAR_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            load(name);
            
            for i = 1:6
                subplot(3,2,i)
                if isfield(result,'X_stoch_perturbed_3')
                    spm1d.plot.plot_meanSD((squeeze(result.X_stoch_perturbed_3(i,:,:))-result.X(i,:)')','color',cs(2,:)); hold on
                    title(titles(i));
                end
                ylim([-0.06 0.06])
                xlim([17 47])
                box off             
            end
            
            
            name = ['result_time_0.8_OBSTACLE_forceField_0_' num2str(wM_std) '_' num2str(wPq_std) '_' num2str(wPqdot_std) '.mat'];
            
            load(name);
            
            for i = 1:6
                subplot(3,2,i)
                if isfield(result,'X_stoch_perturbed_3')
                    spm1d.plot.plot_meanSD((squeeze(result.X_stoch_perturbed_3(i,:,:))-result.X(i,:)')','color',cs(3,:)); hold on
                    title(titles(i));
                end
                ylim([-0.06 0.06])
                xlim([17 47])
                box off
            end           
        end
    end
end


