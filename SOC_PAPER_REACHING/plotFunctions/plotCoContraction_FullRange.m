clear all; close all; clc;
figure
subplot(1,2,1)
[a1,a2] = meshgrid(0.001:0.01:1);
co_contraction = computeCocontraction_Rudolph(a1,a2);
mesh(a1,a2,co_contraction);
colormap(hot)
axis equal
xlabel('a1'); ylabel('a2')

subplot(1,2,2)
[a1,a2] = meshgrid(0.001:0.01:1);
co_contraction = computeCocontraction_Falconer(a1,a2);
mesh(a1,a2,co_contraction);
colormap(hot)
axis equal
xlabel('a1'); ylabel('a2')

figure
subplot(1,2,1)
[a1,a2] = meshgrid(0.001:0.01:1);
co_contraction = computeCocontraction_Rudolph(a1,a2);
contourf(a1,a2,co_contraction);
colormap(hot)
axis equal
xlabel('a1'); ylabel('a2')

subplot(1,2,2)
[a1,a2] = meshgrid(0.001:0.01:1);
co_contraction = computeCocontraction_Falconer(a1,a2);
contourf(a1,a2,co_contraction);
colormap(hot)
axis equal
xlabel('a1'); ylabel('a2')
