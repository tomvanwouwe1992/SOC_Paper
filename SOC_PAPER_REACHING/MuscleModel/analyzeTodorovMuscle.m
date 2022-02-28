clc; clear all; close all;
%%% Plotting Todorov & Li muscle model properties

% Muscle tension :: T(a,l,v) = A(a,l)(F_l(l)*F_V(l,v)+F_P(l))

a = [0 1];
l = [0.2 1.3];
v = [-2 2];

% Interpolate variables with 100 intervals (101 datapoints)
a = interp1([0,1],a,0:0.01:1);
l = interp1([0,1],l,0:0.01:1);
v = interp1([0,1],v,0:0.01:1);

% Activation :: A(a,l)
% Depends on the length as well ~ number of fibers
figure(1)
[a_grid,l_grid] = meshgrid(a,l);
meshc(a_grid, l_grid, ActivationFunction(a_grid,l_grid))
title('Activation')
xlabel('a')
ylabel('l')

% Activation :: A(a,l)
% Depends on the length as well ~ number of fibers
figure(1)
subplot(2,2,1)
[a_grid,l_grid] = meshgrid(a,l);
meshc(a_grid, l_grid, ActivationFunction(a_grid,l_grid))
title('Activation')
xlabel('a')
ylabel('l')

% Force-Length :: F_L(l)
subplot(2,2,2)
plot(l,ForceLength(l))
title('F_L')
xlabel('l')

% Force-Velocity :: F_V(l,v)
subplot(2,2,3)
[l_grid,v_grid] = meshgrid(l,v);
meshc(l_grid, v_grid, ForceVelocity(l_grid,v_grid))
title('F_V')
xlabel('l')
ylabel('v')

% Force passive :: F_P(l)
subplot(2,2,4)
plot(l, ForcePassive(l));
title('F_P (something wrong?)')
xlabel('l')


function Activation = ActivationFunction(a,l)
N_f = 2.11 + 4.16*(1./l-1);
Activation = 1 - exp(-(a./(0.56*N_f)).^N_f);
end

function F_L = ForceLength(l)
F_L = exp(-abs((l.^1.93-1)/1.03).^1.87);
end

function F_V = ForceVelocity(l,v)

F_V_min = (-5.72 - v)./(-5.72 + (1.38+2.09.*l).*v);


F_V_plus = (0.62-(-3.12+4.21.*l-2.67.*l.^2).*v)./(0.62+v);

F_V = heaviside(-v).*F_V_min + heaviside(v).*F_V_plus;

end

function F_P = ForcePassive(l)
F_P = 0.02*exp(-13.8+18.7*(l));
end