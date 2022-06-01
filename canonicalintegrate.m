function [xs, vs] = canonicalintegrate(time,dt,time_exec,order)
% Integrate a canonical system for 'ceil(1+time_exec/dt)' time steps of duration 'dt'
%
% This function uses the closed-form solution of the dynamical system
% representing the canonical system.
%
% Input:
%   time          - duration of the observed movement
%   dt            - duration of the integration step
%   time_exec     - duration of the integration
%   order         - order of the canonical system (1 or 3)
%   alpha         - time constant, determined speed of convergence
% Output:
%   ts            - time over time, i.e. 0.01, 0.02, 0.03 etc
%   xs,xds,vs,vds - state of the canonical system over time
%   alpha         - time constant, determines speed of convergence

if (nargin==0)
  % If no arguments are passed, test the function
  [ts, xs, xds, vs, vds, alpha] = testcanonicalintegrate;
  return;
end

% Set some defaults
if (nargin<3), time_exec = time; end
if (nargin<4), order = 2; end
if (nargin<5)
  if (order==1)
    % Set alpha such that the canonical system is 0.001 at the end of the
    % movement
    alpha = -log(0.001);
  else
    % Set alpha to 15, as we always do.
    alpha = 15;
  end
end

N = ceil(1+time_exec/dt); % Number of time steps
ts = dt*(0:N-1)';% time over time
if (order==1)
  % Closed form solution to 1st order canonical system
  xs  = exp(-alpha*ts./time);
  xds = -(alpha/time)*exp(-alpha*ts/time);
  vs = xs;
  vds = xds;

end



