clear all; close all; clc;

dataVector = NaN(7,8);
cs = linspecer(4,'sequential');

localNames = {'solution_SWAYCTR_ROTATION','solution_SRS_SWAYCTR_ROTATION','solution_VESTIBULARLOSS_SWAYCTR_ROTATION','solution_SRS_VESTIBULARLOSS_SWAYCTR_ROTATION'};


for j = 1:4
    localName = localNames(j);
    load([char(localName) '_wMx1_wPx' num2str(1) '_wVx' num2str(1) '.mat']);
    ct = 1;
    for i = 1:7
        out = solutions(i).out;
        Krel = out.Kopt(1,1)/(out.Kopt(1,1) + out.Kopt(3,1));
        CCI = out.a_base_opt(1);
        RMS_sway = 180/pi*sqrt(out.P_opt(3,3)+out.P_opt(5,5)+2*out.P_opt(3,5));
        corr = out.P_opt(3,5)/(sqrt(out.P_opt(3,3))*sqrt(out.P_opt(5,5)));
        FF = out.J_baseline_activation_opt;
        FB = out.J_fb_proprio_accuracy_opt + out.J_fb_proprio_state_opt + out.J_fb_vestibular_accuracy_opt + out.J_fb_vestibular_state_opt;
        Cost = FF+FB + 10*(RMS_sway*pi/180)^2;
        dataVector(ct,:) = [180/pi*out.wPq_Platform_std RMS_sway 180/pi*sqrt(out.P_opt(3,3))  Krel  CCI  FB/(FB+FF) Cost  out.convergence ];
        ct = ct+1;
        
    end
    
    
    for plot_index = 1:6
        subplot(1,6,plot_index)
        
        if plot_index == 1
            for lg = 1:4
                scatter(0,-100,'MarkerFaceColor',cs(lg,:),'MarkerEdgeColor',cs(lg,:)); hold on;
            end
            legend('healthy','healthy SRS','VL','VL SRS')
            
        end
        
        box off
        if sum(dataVector(:,end),1) == 7
            plot(dataVector(:,1),dataVector(:,plot_index+1),'Color',cs(j,:),'LineWidth',1); hold on;
            for s = 1:7
                if dataVector(s,end) == 1
                    scatter(dataVector(s,1),dataVector(s,plot_index+1),'MarkerFaceColor',cs(j,:),'MarkerEdgeColor',cs(j,:));
                else
                    scatter(dataVector(s,1),dataVector(s,plot_index+1),'d','MarkerFaceColor',cs(j,:),'MarkerEdgeColor',cs(j,:));
                end
            end
            if plot_index == 1
                ylim([0 8]);
                title('Body sway')
                xlabel('Rotation magnitude')
                ylabel('[°]')
            elseif plot_index == 2
                ylim([0 8]);
                ylabel('[°]')
                title('Ankle sway')
                xlabel('Rotation magnitude')
                
                
                
            elseif plot_index == 3
                
                ylim([-0.1 1]);
                title('K_{rel,p}')
                xlabel('Rotation magnitude')
                ylabel('[-]')
                
            elseif plot_index == 4
                ylim([0 1]);
                title('CCI')
                xlabel('Rotation magnitude')
                ylabel('[-]')
                
            elseif plot_index == 5
                ylim([0 1]);
                title('FB contribution')
                xlabel('Rotation magnitude')
                ylabel('[-]')
            else
                
                
                ylim([-1 1]);
                title('CorrelationCoeff ankle-platform')
                xlabel('Rotation magnitude')
                ylabel('[-]')
                
            end
        end
    end
end