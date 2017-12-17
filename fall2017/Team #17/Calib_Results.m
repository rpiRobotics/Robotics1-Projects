% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 195.273566614158260 ; 229.300191770569480 ];

%-- Principal point:
cc = [ 272.350082835571070 ; 179.269375617523000 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.276709829110065 ; 0.052107920918756 ; 0.012341529952631 ; -0.001385780176166 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.580426738157296 ; 2.766766229499306 ];

%-- Principal point uncertainty:
cc_error = [ 3.907575787692970 ; 9.591847480946314 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012564408131025 ; 0.005746391175223 ; 0.008009232717845 ; 0.001126868081009 ; 0.000000000000000 ];

%-- Image size:
nx = 560;
ny = 420;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 15;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.297408e+00 ; 1.537353e+00 ; -1.154013e+00 ];
Tc_1  = [ -1.250403e+02 ; 2.038123e+01 ; 3.233008e+02 ];
omc_error_1 = [ 2.530627e-02 ; 3.396395e-02 ; 2.773547e-02 ];
Tc_error_1  = [ 6.314193e+00 ; 1.374530e+01 ; 4.182754e+00 ];

%-- Image #2:
omc_2 = [ 1.488344e+00 ; 1.384077e+00 ; -9.671095e-01 ];
Tc_2  = [ -6.491784e+01 ; 3.043171e+00 ; 1.917547e+02 ];
omc_error_2 = [ 3.041577e-02 ; 3.405059e-02 ; 3.278602e-02 ];
Tc_error_2  = [ 3.871184e+00 ; 8.144304e+00 ; 3.213353e+00 ];

%-- Image #3:
omc_3 = [ 1.478248e+00 ; 1.375700e+00 ; -9.748555e-01 ];
Tc_3  = [ -6.495386e+01 ; 3.504838e+00 ; 1.924692e+02 ];
omc_error_3 = [ 2.922984e-02 ; 3.213214e-02 ; 2.977981e-02 ];
Tc_error_3  = [ 3.845542e+00 ; 8.149699e+00 ; 2.807558e+00 ];

%-- Image #4:
omc_4 = [ 1.731771e+00 ; 7.420269e-01 ; -5.113975e-01 ];
Tc_4  = [ -6.826718e+01 ; 6.906065e+01 ; 1.477438e+02 ];
omc_error_4 = [ 3.989828e-02 ; 2.278038e-02 ; 2.951783e-02 ];
Tc_error_4  = [ 3.157719e+00 ; 7.011399e+00 ; 3.052553e+00 ];

%-- Image #5:
omc_5 = [ 9.571919e-01 ; 1.920011e+00 ; -1.404445e+00 ];
Tc_5  = [ -3.088392e+01 ; 5.306822e+01 ; 2.012050e+02 ];
omc_error_5 = [ 2.346125e-02 ; 4.268460e-02 ; 4.077522e-02 ];
Tc_error_5  = [ 4.103760e+00 ; 8.973198e+00 ; 2.828386e+00 ];

%-- Image #6:
omc_6 = [ 7.618786e-01 ; 2.146224e+00 ; -1.545709e+00 ];
Tc_6  = [ -6.603976e+00 ; 1.216400e+01 ; 3.007249e+02 ];
omc_error_6 = [ 1.601370e-02 ; 4.126600e-02 ; 3.922059e-02 ];
Tc_error_6  = [ 5.937934e+00 ; 1.269869e+01 ; 3.760677e+00 ];

%-- Image #7:
omc_7 = [ 9.467965e-01 ; 1.992212e+00 ; -1.285674e+00 ];
Tc_7  = [ -2.980760e+01 ; 1.232034e+01 ; 2.411477e+02 ];
omc_error_7 = [ 2.000544e-02 ; 3.829847e-02 ; 3.778033e-02 ];
Tc_error_7  = [ 4.776822e+00 ; 1.026295e+01 ; 3.260675e+00 ];

%-- Image #8:
omc_8 = [ 1.327639e+00 ; 1.241916e+00 ; -1.021331e+00 ];
Tc_8  = [ -1.250069e+02 ; 8.497212e+01 ; 3.390353e+02 ];
omc_error_8 = [ 2.964647e-02 ; 3.090760e-02 ; 2.650102e-02 ];
Tc_error_8  = [ 6.838175e+00 ; 1.489498e+01 ; 4.787573e+00 ];

%-- Image #9:
omc_9 = [ 2.863091e+00 ; 2.771471e-01 ; 7.953961e-02 ];
Tc_9  = [ -1.670398e+02 ; 1.193320e+02 ; 2.090537e+02 ];
omc_error_9 = [ 1.997347e-02 ; 5.091374e-03 ; 2.841750e-02 ];
Tc_error_9  = [ 4.611258e+00 ; 1.003206e+01 ; 4.703425e+00 ];

%-- Image #10:
omc_10 = [ 1.622946e+00 ; 1.757666e+00 ; -8.049029e-01 ];
Tc_10  = [ -6.430809e+01 ; -2.104300e+01 ; 4.681986e+02 ];
omc_error_10 = [ 2.671248e-02 ; 3.066422e-02 ; 3.752401e-02 ];
Tc_error_10  = [ 9.304648e+00 ; 1.948986e+01 ; 8.071787e+00 ];

%-- Image #11:
omc_11 = [ 1.844648e+00 ; 1.849997e+00 ; -6.266730e-01 ];
Tc_11  = [ -1.086982e+02 ; -8.847393e+00 ; 3.659006e+02 ];
omc_error_11 = [ 2.445781e-02 ; 2.721538e-02 ; 3.838682e-02 ];
Tc_error_11  = [ 7.201265e+00 ; 1.529325e+01 ; 6.800308e+00 ];

%-- Image #12:
omc_12 = [ 1.569645e+00 ; 1.713672e+00 ; -6.990498e-01 ];
Tc_12  = [ -9.847928e+01 ; -2.100008e+00 ; 3.473642e+02 ];
omc_error_12 = [ 2.851745e-02 ; 3.167414e-02 ; 3.875517e-02 ];
Tc_error_12  = [ 6.916884e+00 ; 1.469175e+01 ; 6.242951e+00 ];

%-- Image #13:
omc_13 = [ 1.171317e+00 ; 2.411730e+00 ; -7.931764e-01 ];
Tc_13  = [ -4.838162e+01 ; -4.594673e+01 ; 3.190542e+02 ];
omc_error_13 = [ 2.051960e-02 ; 4.190718e-02 ; 5.547383e-02 ];
Tc_error_13  = [ 6.474327e+00 ; 1.308479e+01 ; 6.294061e+00 ];

%-- Image #14:
omc_14 = [ 5.058373e-01 ; -2.136009e+00 ; 6.716118e-01 ];
Tc_14  = [ 1.808553e+02 ; 2.860139e+01 ; 3.395166e+02 ];
omc_error_14 = [ 2.392659e-02 ; 3.586807e-02 ; 5.447560e-02 ];
Tc_error_14  = [ 7.414037e+00 ; 1.502007e+01 ; 8.275679e+00 ];

%-- Image #15:
omc_15 = [ 1.595502e+00 ; 1.558173e+00 ; -1.469239e+00 ];
Tc_15  = [ -2.027346e+01 ; 1.629685e+01 ; 3.662148e+02 ];
omc_error_15 = [ 2.351223e-02 ; 4.016278e-02 ; 3.000004e-02 ];
Tc_error_15  = [ 7.291153e+00 ; 1.526322e+01 ; 4.072783e+00 ];

