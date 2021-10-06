function auxdata = configure_inputs(auxdata)
% ===================== Model Configuration ======================= 
% Model configuration function to configure models used in the 
%                    trajectory optimisation.                          
% ================================================================= 
%
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
% Earth radius model
% --------------------------------------
% 1: constant radius model
% 2: Geodedic earth model
%
% Earth rotation model
% --------------------------------------
% 0: no rotation
% 1: rotation

auxdata.gravity_model = 1;
auxdata.constant_mass = 1;
auxdata.radius_model = 1;
auxdata.earth_rotation = 0;