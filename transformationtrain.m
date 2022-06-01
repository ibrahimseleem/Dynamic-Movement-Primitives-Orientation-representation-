function theta= transformationtrain(trajectory,n_basis_functions,xs,vs,time)

% Compute basis function activations

  % Time signal is phase
  ps = xs;
  % Reconstruct alpha
  dt=mean(diff(trajectory.t));
  alpha = -time*log(xs(2))/dt;
  % Get centers and widths
  [centers,widths] = basisfunctioncenters(n_basis_functions,time,alpha);

% Compute activations
activations = basisfunctionactivations(centers,widths,ps);


%%
% the following gain values are optimized based on the application
alpha_z=348.348845225845;
beta_z=977.240647794841;


tau=trajectory.t(end);


f_target=((trajectory.D0)\((tau.*trajectory.eta_dot)+(alpha_z*trajectory.eta)-(alpha_z*beta_z.*trajectory.omega))')';


% f_target=f_target./xs;
%-------------------------------------------------------------------------------
% Compute the regression, using linear least squares
%   (http://en.wikipedia.org/wiki/Linear_least_squares)
vs_repmat = repmat(vs',n_basis_functions,1);
sum_activations = repmat(sum(abs(activations),2)',n_basis_functions,1);
activations_normalized = activations' ./ sum_activations;
vs_activ_norm = vs_repmat.*activations_normalized;
small_diag = diag(ones(n_basis_functions,1)*1e-10);
AA = inv(vs_activ_norm*vs_activ_norm' + small_diag)*vs_activ_norm ;
theta = (AA * f_target)';

end