function auxdata = configure_inputs(auxdata)
% ===================== Model Configuration ======================= 
% Model configuration function to configure models used in the 
%                    trajectory optimisation.                          
% ================================================================= 
% Gravity models
% --------------------------------------
% 1: constant gravity model 
% 2: flat earth model
% 3: spherical earth model (incomplete)
% 4: ellipsoid model
% 5: fourth order zonal harmonics model
% 
% Constant mass model
% --------------------------------------
% 0: variable mass
% 1: constant mass (initial mass)
% 
% 
% 
% 

auxdata.gravity_model = 4;
auxdata.constant_mass = 0;
