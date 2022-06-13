% This function is used to compute the desired reference orientation values


function trajectory = generateTrajquat()


% load unit quaternion orientation. 
% this is real extracted orientation data from robotic manipulator.

load('quaternion_orientation');

trajectory.t = data(:,1); % time

trajectory.q=data(:,2:end); % desired unit quaternion orientation

trajectory.g0=trajectory.q(end,:); % desired goal orientation

trajectory.tau=trajectory.t(end); % Time scaling constant

%% calculation OF OMEGA && ETA


x=zeros(length(trajectory.t),length(trajectory.q(1,:)));

for j=1:1:length(trajectory.t)
      
        x(j,:)=2*quatlog(quatnormalize(quatmultiply(trajectory.g0,quatconj(trajectory.q(j,:)))));  % equation (23) 
   
        trajectory.omega(j,:)=x(j,2:4); % extract desired vector omega
   
        trajectory.eta(j,:)=trajectory.tau*trajectory.omega(j,:); % calculate desired eta

end


eta_dot_old=[0 0 0];   

trajectory.eta_dot=[eta_dot_old;trajectory.tau*diff(trajectory.eta)]; % desired eta_dot


trajectory.R0=quat2rotm(trajectory.q(1,:)); % desired initial orientation
trajectory.Rg=quat2rotm(trajectory.q(end,:)); % desired goal orientation

trajectory.D0=diag(vex(logm(trajectory.Rg*trajectory.R0'))); % calculation OF D0 in equation (21)

end
