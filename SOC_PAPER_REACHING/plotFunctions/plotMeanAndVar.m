function [] = plotMeanAndVar(time,meanTraj,stdTraj,color)

    plot(time,meanTraj,'LineWidth',2,'Color',color); hold on;
    plot(time,meanTraj+stdTraj,'--','LineWidth',1,'Color',color);
    plot(time,meanTraj-stdTraj,'--','LineWidth',1,'Color',color);
end
