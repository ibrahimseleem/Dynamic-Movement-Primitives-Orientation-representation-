% Author:  Ibrahim A. Seleem
% Website: https://orcid.org/0000-0002-3733-4982

% This code is mofified based on different resources including

%[1] "dmp_bbo: Matlab library for black-box optimization of dynamical movement primitives.", Freek Stulp, Robotics and Computer Vision, ENSTA-ParisTech
%[2] article : "Orientation in cartesian space dynamic movement primitives."
%[3] Seleem, I. A., El-Hussieny, H., Assal, S. F., & Ishii, H. "Development and stability analysis of an imitation learning-based pose planning approach for multi-section continuum robot."
%[4] Seleem, I. A., Assal, S. F., Ishii, H., & El-Hussieny, H. "Guided pose planning and tracking for multi-section continuum robots considering robot dynamics."

% If you use this code in the context of a publication, I would appreciate 
% it if you could cite our previous work as follows:
%

% @article{seleem2019guided,
%   title={Guided pose planning and tracking for multi-section continuum robots considering robot dynamics},
%   author={Seleem, Ibrahim A and Assal, Samy FM and Ishii, Hiroyuki and El-Hussieny, Haitham},
%   journal={IEEE Access},
%   volume={7},
%   pages={166690--166703},
%   year={2019},
%   publisher={IEEE}
% }

% @article{seleem2020development,
%   title={Development and stability analysis of an imitation learning-based pose planning approach for multi-section continuum robot},
%   author={Seleem, Ibrahim A and El-Hussieny, Haitham and Assal, Samy FM and Ishii, Hiroyuki},
%   journal={IEEE Access},
%   volume={8},
%   pages={99366--99379},
%   year={2020},
%   publisher={IEEE}
% }


clear;
close all;
clc;



trajectory=generateTrajquat();

[theta,trajectory,trajectory3] = dmptrain(trajectory,1,100); % 1-order, 100= no. of basis functions

