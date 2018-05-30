clear all
%% run first iteration
  theta_0 = zeros(1,10);
  theta_0(1,1) = -.1;
  theta_0(1,4) = 1;
  csvwrite('data/0_theta.csv',theta_0);
%   duration = 1;
  length_vec = .15:.05:.35;
  duration_vec = [.5 1];
  for batch=1:10,
    length_ind = mod(batch-1,5)+1;
    duration_ind = floor((batch-1)/5) + 1;
      system(sprintf('./runSGDIter -length %.2f -duration %.2f -init init_length_%d_speed_0_z.csv -weights 0_theta.csv -prefix 0_%d_',...
    length_vec(length_ind), duration_vec(duration_ind), length_ind-1,batch-1));
  end

%% run later iterations
for iter=21:50,
  %% 
  for batch=1:10,
    B{batch} = load(sprintf('data/%d_%d_B.csv',iter-1,batch-1));
    A{batch} = load(sprintf('data/%d_%d_A.csv',iter-1,batch-1));
    H{batch} = load(sprintf('data/%d_%d_H.csv',iter-1,batch-1));
    lb{batch} = load(sprintf('data/%d_%d_lb.csv',iter-1,batch-1));
    y{batch} = load(sprintf('data/%d_%d_y.csv',iter-1,batch-1));
    ub{batch} = load(sprintf('data/%d_%d_ub.csv',iter-1,batch-1));
    w{batch} = load(sprintf('data/%d_%d_w.csv',iter-1,batch-1));
    z{batch} = load(sprintf('data/%d_%d_z.csv',iter-1,batch-1));
    theta{batch} = load(sprintf('data/%d_theta.csv',iter-1));
    
    constraint_inds = y{batch} >= ub{batch} - 1e-3 | y{batch} <= lb{batch} + 1e-3;
    A_active{batch} = A{batch}(constraint_inds,:);
    B_active{batch} = B{batch}(constraint_inds,:);
  end
  AB = [blkdiag(A_active{:}) vertcat(B_active{:})];
  nt = numel(theta{1});
  nz = size(AB,2)-nt;
  w_ext = [vertcat(w{:});zeros(nt,1)];
  %%
  
%   N = null(AB);
%   
%   
%   grad_null = N*N'*w_ext;
%   dtheta_null = grad_null(nz+1:end);
%   dtheta_null = reshape(dtheta_null,[],size(theta{1},1))';
  
  %%
  nl = size(AB,1);
  Q=[eye(nz+nt) AB'; AB zeros(nl)];
  Q=[blkdiag(H{:},.01*eye(nt)) AB'; AB zeros(nl)];
  
  b = [w_ext;zeros(nl,1)];
  grad_solve = Q\b;
  grad_solve = grad_solve(1:nz+nt);
  dtheta_solve = -reshape(grad_solve(nz+1:end),[],size(theta{1},1))';
  
  grad_check = w_ext'*grad_solve/norm(grad_solve);
  cstr_check = norm(AB*grad_solve);
  display(sprintf('Iteration: %d \t Gradient:%.3f \t Constraint: %.2e',iter, grad_check,cstr_check));
  
  scale = .9^iter;
  dtheta = scale*dtheta_solve/sqrt(grad_solve'*blkdiag(H{:},.01*eye(nt))*grad_solve);
  %%
  csvwrite(sprintf('data/%d_theta.csv',iter),theta{1} + dtheta)
  
  duration = 1;
  length_vec = .15:.05:.35;
  for batch=1:10,
    length_ind = mod(batch-1,5)+1;
    duration_ind = floor((batch-1)/5) + 1;
      system(sprintf('./runSGDIter -length %.2f -duration %.2f -init %d_%d_z.csv -weights %d_theta.csv -prefix %d_%d_',...
    length_vec(length_ind), duration_vec(duration_ind), iter-1 ,batch-1, iter, iter, batch-1));
  end
end

