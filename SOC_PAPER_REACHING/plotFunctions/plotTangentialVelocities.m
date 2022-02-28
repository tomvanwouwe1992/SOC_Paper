function [] = plotTangentialVelocities(result)

EEVel = result.EEVel;
tangentialVel = sqrt(EEVel(:,1).^2 + EEVel(:,2).^2);
plot(result.time,tangentialVel);