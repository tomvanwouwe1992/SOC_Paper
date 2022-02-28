clear all; close all;

ct_fig = 0;

dataVector = NaN(7,6);
ct = 1;
cs = linspecer(4,'sequential');
for k = 0:2:4
    ct_fig = ct_fig + 1;
            figure(ct_fig)

    for j = 0:2:6
        wP_ = 2^(j);
        wV_ = 2^(k);
        load(['solution_SWAYCTR_TRANSLATION_wMx1_wPx' num2str(wP_) '_wVx' num2str(wV_) '.mat']);
        
        for i = 1:7
            out = solutions(i).out;
            Krel = out.Kopt(1,1)/(out.Kopt(1,1) + out.Kopt(3,1));
            CCI = out.a_base_opt(1);
            RMS_sway = 180/pi*sqrt(out.Pmat_opt(3,3));
%             corr = out.P_opt(3,5)/(sqrt(out.P_opt(3,3))*sqrt(out.P_opt(5,5)));
            FF = sumsqr(out.a_base_opt);
            FB = out.J_fb_proprio_accuracy_opt + out.J_fb_proprio_state_opt + out.J_fb_vestibular_accuracy_opt + out.J_fb_vestibular_state_opt;
            dataVector(ct,:) = [180/pi*out.wqPdotdot_Platform_std RMS_sway  Krel  CCI  FB/(FB+FF)  out.convergence ];
            ct = ct+1;
            
        end
        
        for plot_index = 1:4
            subplot(1,4,plot_index)
            box off
            if sum(dataVector(:,end),1) == 7
                plot(dataVector(:,1),dataVector(:,plot_index+1),'Color',cs(j/2+1,:),'LineWidth',1); hold on;
                for s = 1:7
                    if dataVector(s,end) == 1
                        scatter(dataVector(s,1),dataVector(s,plot_index+1),'MarkerFaceColor',cs(j/2+1,:),'MarkerEdgeColor',cs(j/2+1,:));
                    else
                        scatter(dataVector(s,1),dataVector(s,plot_index+1),'d','MarkerFaceColor',cs(j/2+1,:),'MarkerEdgeColor',cs(j/2+1,:));
                    end
                end
                if plot_index == 1
                    ylim([0 8]);
                    title('Body sway')
                    xlabel('Perturbation magnitude')
                    ylabel('[°]')
                    
                    
                    
                elseif plot_index == 2
                    
                    ylim([-0.1 1]);
                    title('K_{rel,p}')
                    xlabel('Perturbation magnitude')
                    ylabel('[-]')
                    
                elseif plot_index == 3
                    ylim([0 1]);
                    title('CCI')
                    xlabel('Perturbation magnitude')
                    ylabel('[-]')
               
                else 
                    ylim([0 1]);
                    title('FB contribution')
                    xlabel('Perturbation magnitude')
                    ylabel('[-]')
                    
                end
            end
        end
        ct = 1;
        
    end
end

