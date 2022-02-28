clear all; close all; clc;

dataVector = NaN(7,8);
cs = linspecer(4,'sequential');

localNames = {'solution_SWAYCTR_TRANSLATION','solution_SRS_SWAYCTR_TRANSLATION','solution_VESTIBULARLOSS_SWAYCTR_TRANSLATION','solution_SRS_VESTIBULARLOSS_SWAYCTR_TRANSLATION'};


for j = 1:4
    localName = localNames(j);
    load([char(localName) '_wMx1_wPx' num2str(1) '_wVx' num2str(1) '.mat']);
    ct = 1;
    for i = 1:7
        out = solutions(i).out;
        Krel = out.Kopt(1,1)/(out.Kopt(1,1) + out.Kopt(3,1));
        CCI = out.a_base_opt(2) - 0.0184;
        RMS_sway = 180/pi*sqrt(out.Pmat_opt(3,3));
        corr = 0;
        FF = sumsqr(out.a_base_opt);
        FB = out.J_fb_proprio_accuracy_opt + out.J_fb_proprio_state_opt + out.J_fb_vestibular_accuracy_opt + out.J_fb_vestibular_state_opt;
        totalCost = FF + FB + 10*out.Pmat_opt(3,3);
        
        dataVector(ct,:) = [180/pi*out.wPq_Platform_std RMS_sway 180/pi*sqrt(out.Pmat_opt(3,3))  Krel  CCI  FB/(FB+FF)  FF + FB  out.convergence ];
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
                xlabel('Translation magnitude')
                ylabel('[°]')
            elseif plot_index == 2
                ylim([0 8]);
                ylabel('[°]')
                title('Ankle sway')
                xlabel('Translation magnitude')
                
                
                
            elseif plot_index == 3
                
                ylim([-0.1 1]);
                title('K_{rel,p}')
                xlabel('Translation magnitude')
                ylabel('[-]')
                
            elseif plot_index == 4
                ylim([0 1]);
                title('CCI')
                xlabel('Translation magnitude')
                ylabel('[-]')
                
            elseif plot_index == 5
                ylim([0 1]);
                title('FB contribution')
                xlabel('Translation magnitude')
                ylabel('[-]')
            else
                
                
%                 ylim([-1 1]);
                title('CorrelationCoeff ankle-platform')
                xlabel('Rotation magnitude')
                ylabel('[-]')
                
            end
        end
    end
end

