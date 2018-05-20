clear all

B = load('B.csv');
A = load('A.csv');
H = load('H.csv');
lb = load('lb.csv');
y = load('y.csv');
ub = load('ub.csv');
w = load('w.csv');
z = load('z.csv');

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
[X,FVAL,FLAG] = quadprog(Q,b,[M;-M],5e-2*ones(2*nz+2*nt,1));



X_full = M*X;
dz = X_full(1:nz);
dtheta = X_full(nz+1:end);


FVAL
norm(dtheta)

if exist('dtheta.csv')
 dtheta0 = load('dtheta.csv');
 csvwrite('dtheta.csv',dtheta0 + dtheta) 
else
 csvwrite('dtheta.csv',0.1*dtheta) 
end


 