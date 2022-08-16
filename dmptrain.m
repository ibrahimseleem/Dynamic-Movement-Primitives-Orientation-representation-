function [ theta,trajectory,trajectory3] = dmptrain(trajectory,order,n_basis_functions)
       

time= trajectory.t(end);
time_exec = trajectory.t(end);
dt = mean(diff(trajectory.t)); % sampling time


[xs,vs] = canonicalintegrate(time,dt,time_exec,order); % function to compute phase variable in equation (5)


theta= transformationtrain(trajectory,n_basis_functions,xs,vs,time);  %% calculate the desired force function Equation (8)

  
 
n_basis_functions = length(theta);

%-------------------------------------------------------------------------------
% Duration of the motion in time steps
T = length(xs);

%-------------------------------------------------------------------------------
% Compute basis function activations

  % Time signal is phase
  ps = xs;
  % Reconstruct alpha
  alpha = -time*log(xs(2))/dt;
  % Get centers and widths
  [centers,widths] = basisfunctioncenters(n_basis_functions,time,alpha);

% Compute activations
activations = basisfunctionactivations(centers,widths,ps);

% Compute sum(phi*wi)/sum(phi)
weighted_sum_activations = sum(activations*theta',3);
sum_activations =sum (activations,2);
f= (trajectory.D0*((weighted_sum_activations./sum_activations).*vs)')'; 


% the following gain values are optimized based on the application
alpha_z=348.348845225845;
beta_z=977.240647794841;
alphag0=diag([566.708977959913,1122.83972710486,1175.67591183085,1122.18942064066]); %% optimized gain 


tau=time; % phase variable gain in equation (5)
trajectory3.eta(1,:)=[trajectory.eta(1,1) trajectory.eta(1,2) trajectory.eta(1,3)];
trajectory3.omega(1,:)=[trajectory.omega(1,1) trajectory.omega(1,2) trajectory.omega(1,3)]; % initial condition of Omega
trajectory3.q(1,:)=trajectory.q(1,:); % initial quaternion output 
trajectory3.g0=trajectory.g0;         % initial output goal


for kk=2:T
    
    eta_rdot=((alpha_z*((beta_z*trajectory3.omega(kk-1,:))-trajectory3.eta(kk-1,:)))+f(kk,:))/tau; % compute output eta_dot euqation (16)

    trajectory3.eta(kk,:)=trajectory3.eta(kk-1,:)+(dt*eta_rdot); % integrate equation (16) to get output eta 

    r= (dt*[0 trajectory3.eta(kk,:)])/(2*tau); % equation (17)
   
    trajectory3.q(kk,:)= quatmultiply(quatexp(r),trajectory3.q(kk-1,:));     %% integration of equation (17), integration result is equation (24)        
    
    r1= (dt*alphag0*(quatlog(quatnormalize(quatmultiply(trajectory.g0,quatconj(trajectory3.g0)))))')/(tau);
   
    trajectory3.g0= quatmultiply(quatexp(r1'),trajectory3.g0);

    
    omega_eq=2*quatlog(quatnormalize(quatmultiply(trajectory3.g0,quatconj(trajectory3.q(kk,:)))));  % equation (23)
    trajectory3.omega(kk,:)=omega_eq(2:4);                   % compyte Omega
    trajectory3.eta(kk,:)=tau*trajectory3.omega(kk,:);   % compute eta
    
end
  
  
  
  
%% Graphs 

subplot(2,2,1);
grid on
plot(trajectory.t,trajectory3.q(:,1),'-r','LineWidth',2)
hold on 
plot(trajectory.t,trajectory.q(:,1),'Color','k','MarkerSize',15,'DisplayName','Goal');
grid on
xlabel('Time [sec.]','FontSize',12,'FontWeight','bold')
ylabel('$S$','interpreter','latex','FontSize',12,'FontWeight','bold')
legend({'Ref. $S$','Learn $S$'},'interpreter','latex','FontSize',9,'FontWeight','bold','Location','northwest')

subplot(2,2,2);
grid on
plot(trajectory.t,trajectory3.q(:,2),'-r','LineWidth',2)
hold on 
plot(trajectory.t,trajectory.q(:,2),'Color','k','MarkerSize',15,'DisplayName','Goal');
grid on
xlabel('Time [sec.]','FontSize',12,'FontWeight','bold')
ylabel('$v_1$','interpreter','latex','FontSize',12,'FontWeight','bold')
legend({'Ref. $v_1$','Learn $v_1$'},'interpreter','latex','FontSize',9,'FontWeight','bold','Location','southeast')

subplot(2,2,3);
grid on
plot(trajectory.t,trajectory3.q(:,3),'-r','LineWidth',2)
hold on 
plot(trajectory.t,trajectory.q(:,3),'Color','k','MarkerSize',15,'DisplayName','Goal');
grid on
xlabel('Time [sec.]','FontSize',12,'FontWeight','bold')
ylabel('$v_2$','interpreter','latex','FontSize',12,'FontWeight','bold')
legend({'Ref. $v_2$','Learn $v_2$'},'interpreter','latex','FontSize',9,'FontWeight','bold','Location','northwest')

subplot(2,2,4);
grid on
plot(trajectory.t,trajectory3.q(:,4),'-r','LineWidth',2)
hold on 
plot(trajectory.t,trajectory.q(:,4),'Color','k','MarkerSize',15,'DisplayName','Goal');
grid on
xlabel('Time [sec.]','FontSize',12,'FontWeight','bold')
ylabel('$v_3$','interpreter','latex','FontSize',12,'FontWeight','bold')
legend({'Ref. $v_3$','Learn $v_3$'},'interpreter','latex','FontSize',9,'FontWeight','bold','Location','southeast')

end
