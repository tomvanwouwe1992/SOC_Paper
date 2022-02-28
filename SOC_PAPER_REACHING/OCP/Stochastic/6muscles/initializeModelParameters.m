function auxdata = initializeModelParameters()
% Degrees of freedom
auxdata.Ndof=2;

% Movement time
T = 0.8; auxdata.T = T; % time horizon

% Parameters of skeletal arm model
auxdata.m1 = 1.4; auxdata.m2 = 1;
auxdata.l1 = 0.3; auxdata.l2 = 0.33;
auxdata.lc1 = 0.11; auxdata.lc2 = 0.16;
auxdata.I1 = 0.025;
auxdata.I2 = 0.045;

% Load and store coefficients for muscle tendon lengts and moment arms
load('dM_coefficients.mat')
load('LMT_coefficients.mat')

auxdata.dM_coefficients  = dM_coefficients; % First shoulder muscles, then elbow muscles
auxdata.LMT_coefficients = LMT_coefficients; % First shoulder muscles, then elbow muscles

% Muscle parameters (Hill-type)
auxdata.Fiso = 31.8*[ 18; 14; 22; 12; 5; 10];
auxdata.vMtilde_max = [10;10;10;10;10;10];
auxdata.muscleDampingCoefficient = [ 0.01; 0.01; 0.01; 0.01; 0.01; 0.01];
auxdata.tau = 0.150; % Delay

% Parameters of active muscle force-velocity characteristic
load('ActiveFVParameters.mat','ActiveFVParameters');
Fvparam(1) = 1.475*ActiveFVParameters(1); Fvparam(2) = 0.25*ActiveFVParameters(2);
Fvparam(3) = ActiveFVParameters(3) + 0.75; Fvparam(4) = ActiveFVParameters(4) - 0.027;
auxdata.Fvparam = Fvparam;

% Parameters of active muscle force-length characteristic
load('Faparam.mat','Faparam');
auxdata.Faparam = Faparam;

% Parameters of passive muscle force-length characteristic
e0 = 0.6; kpe = 4; t50 = exp(kpe * (0.2 - 0.10e1) / e0);
pp1 = (t50 - 0.10e1); t7 = exp(kpe); pp2 = (t7 - 0.10e1);
auxdata.Fpparam = [pp1;pp2];

auxdata.indices_P = [1 2 3 4 5 6 7 8 9 10; 2 11:19; 3 12 20:27 ; 4 13 21 28:34; 5 14 22 29 35:40; 6 15 23 30 36 41:45; 7 16 24 31 37 42 46:49; 8 17 25 32 38 43 47 50:52; 9 18 26 33 39 44 48 51 53 54;10 19 27 34 40 45 49 52 54 55];
auxdata.varIndices_P = [1 11 20 28 35 41 46 50 53 55 ];
% auxdata.covarIndices_P = [2 3 4 5 6 7 8 10 11 12 13 14 15 17 18 19 20 21 23 24 25 26 28 29 30 32 33 35];

end

