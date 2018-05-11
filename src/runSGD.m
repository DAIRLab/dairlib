clear all

theta = zeros(43,1);
theta(1) = -.1;
theta(6) = 1;
csvwrite('data/0_theta.csv',theta);

system('./sgd_iter -strideLength 0.5 -duration 1 -iter 200 -init z_save.csv -weights 0_theta.csv -prefix 0_');

%%
for iter=1:20,
  B = load(sprintf('data/%d_B.csv',iter-1));
  A = load(sprintf('data/%d_A.csv',iter-1));
  H = load(sprintf('data/%d_H.csv',iter-1));
  lb = load(sprintf('data/%d_lb.csv',iter-1));
  y = load(sprintf('data/%d_y.csv',iter-1));
  ub = load(sprintf('data/%d_ub.csv',iter-1));
  w = load(sprintf('data/%d_w.csv',iter-1));
  z = load(sprintf('data/%d_z.csv',iter-1));
  theta = load(sprintf('data/%d_theta.csv',iter-1));
  
  %%
  constraint_inds = y >= ub - 1e-3 | y <= lb + 1e-3;
  AA = A(constraint_inds,:);
  BB = B(constraint_inds,:);
  % c = AA*z;
  % N=null(AA);
  %
  % grad = w;
  % dz = -N*N'*grad;
  % .5*dz'*H*dz + w'*dz
  % [zz,fval,exitflag] = quadprog(H,w,[],[],AA,zeros(length(c),1));
  % fval
  % zzz=-N*inv(N'*H*N)*N'*w;
  % .5*zzz'*H*zzz + w'*zzz
  
  %%
  h = z(1:19);
  x = z(19 + (1:280));
  x = reshape(x,14,[]);
  u_inds = 280+19+(1:80);
  u = z(u_inds);
  u = reshape(u,4,[]);
  %
  % w'*(N*inv(N'*H*N)*N'*H - eye(length(z)))*pinv(AA)*BB;
  
  %%
  % nc = size(AA,1);
  nz = size(AA,2);
  nt = size(BB,2);
  
  M = null([AA BB]);
  
  Q = M'*[H zeros(nz,nt); zeros(nt,nz) 1e-2*eye(nt)]*M;
  b = M'*[w; zeros(nt,1)];
  [X,FVAL,FLAG] = quadprog(Q,b,[M;-M],5e-1/(5+iter)*ones(2*nz+2*nt,1));
  
  
  
  X_full = M*X;
  dz = X_full(1:nz);
  dtheta = X_full(nz+1:end);
  
  
  FVAL
  norm(dtheta)
    
  csvwrite(sprintf('data/%d_theta.csv',iter),theta + dtheta)
  
  length = 0.5 + .0*rand;
  duration = 1;
  
  system(sprintf('./sgd_iter -strideLength %.2f -duration %.2f -iter 200 -init %d_z.csv -weights %d_theta.csv -prefix %d_',...
    length, duration, iter-1, iter, iter));
end

