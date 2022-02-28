
function auxdata = initializeModelParameters_4muscles_OldModel()


% Degrees of freedom
auxdata.Ndof=2;

% Movement time
T = 0.8; auxdata.T = T; % time horizon

% Parameters of skeletal arm model
auxdata.m1 = 1.4; auxdata.m2 = 1;
auxdata.l1 = 0.3; auxdata.l2 = 0.33;
auxdata.lc1 = 0.11; auxdata.lc2 = 0.16;
auxdata.I1 = auxdata.m1*auxdata.l1^2;
auxdata.I2 = auxdata.m2*auxdata.l2^2;

% Load and store coefficients for muscle tendon lengts and moment arms
load('dM_LMT_params_OldModel.mat')
auxdata.dMcoeff  = [ dM_LMT_params ]; % First shoulder muscles, then elbow muscles
auxdata.LMTcoeff = [ dM_LMT_params ]; % First shoulder muscles, then elbow muscles

% Muscle parameters (Hill-type)
auxdata.Fiso = [ 1142.6; 259.88; 717.5; 525.1];
auxdata.lTs = [ 0.093 ; 0.038;  0.098;  0.2];
% auxdata.lMopt = [0.0901; 0.0503; 0.0979];
auxdata.lMopt = [0.0976 ;0.1367; 0.1138; 0.1157]; % Changed to make a bit more representative of the complete muscle groups
auxdata.alpha = [ 0.3834; 0.3142; 0.1571; 0];
auxdata.muscleDampingCoefficient = [ 0.01; 0.01; 0.01; 0.01];
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

end





