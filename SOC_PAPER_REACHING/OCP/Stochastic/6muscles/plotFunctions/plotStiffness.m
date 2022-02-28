clear all; close all; clc;

loadName = 'result_time_0.8_CIRCLE_forceField_0_0.05_0.0003_0.0024.mat';
stiffnessMatrix_1 = computeStiffness(loadName);


loadName = 'result_time_0.8_BAR_forceField_0_0.05_0.0003_0.0024.mat';
stiffnessMatrix_2 = computeStiffness(loadName);

loadName = 'result_time_0.8_OBSTACLE_forceField_0_0.05_0.0003_0.0024.mat';
stiffnessMatrix_3 = computeStiffness(loadName);

loadName = 'result_time_0.8_CIRCLE_forceField_200_0.05_0.0003_0.0024.mat';
stiffnessMatrix_4 = computeStiffness(loadName);


h = figure('Name','Optimal states, controls and other things');
hTabGroup = uitabgroup;



cs = linspecer(4);



for j = 1:14
    
    tab = uitab(hTabGroup, 'Title', ['time ' num2str(j)]);
    axes('parent',tab);
    
    stiffnessVector = stiffnessMatrix_1(j,:);
    stiffnessVector(stiffnessVector>0)  =0;
    
    for i = 1:size(stiffnessVector,2)
        [x(i),y(i)] = pol2cart((i-1)*pi/16,-stiffnessVector(1,i));
        
    end
    x(i+1) = x(1); y(i+1) = y(1);
%     scatter(x,y,'MarkerFaceColor',cs(1,:),'MarkerEdgeColor',cs(1,:)); hold on;
    plot(x,y,'Color',cs(1,:),'LineWidth',2); hold on;
    
    stiffnessVector = stiffnessMatrix_2(j,:);
    stiffnessVector(stiffnessVector>0)  =0;
    
    
    for i = 1:size(stiffnessVector,2)
        [x(i),y(i)] = pol2cart((i-1)*pi/16,-stiffnessVector(1,i));
        
    end
    x(i+1) = x(1); y(i+1) = y(1);
%     scatter(x,y,'MarkerFaceColor',cs(2,:),'MarkerEdgeColor',cs(2,:)); hold on;
    plot(x,y,'Color',cs(2,:),'LineWidth',2); hold on;
    
    stiffnessVector = stiffnessMatrix_3(j,:);
    stiffnessVector(stiffnessVector>0)  =0;
    
    
    for i = 1:size(stiffnessVector,2)
        [x(i),y(i)] = pol2cart((i-1)*pi/16,-stiffnessVector(1,i));
        
    end
    x(i+1) = x(1); y(i+1) = y(1);
%     scatter(x,y,'MarkerFaceColor',cs(3,:),'MarkerEdgeColor',cs(3,:)); hold on;
    plot(x,y,'Color',cs(3,:),'LineWidth',2); hold on;
    
    stiffnessVector = stiffnessMatrix_4(j,:);
    stiffnessVector(stiffnessVector>0)  =0;
    
    
    for i = 1:size(stiffnessVector,2)
        [x(i),y(i)] = pol2cart((i-1)*pi/16,-stiffnessVector(1,i));
        
    end
    x(i+1) = x(1); y(i+1) = y(1);
%     scatter(x,y,'MarkerFaceColor',cs(4,:),'MarkerEdgeColor',cs(4,:)); hold on;
    plot(x,y,'Color',cs(4,:),'LineWidth',2); hold on;
end