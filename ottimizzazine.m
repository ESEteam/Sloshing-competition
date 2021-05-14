clear 
clc

H = optimvar('H','LowerBound',1,'UpperBound',1.5);
l = optimvar('l','LowerBound',0.4,'UpperBound',1.2);
C = optimvar('C','LowerBound',0.1,'UpperBound',0.5);
M = optimvar('M','LowerBound',0.5,'UpperBound',2);
D = optimvar('D','LowerBound',0.08,'UpperBound',0.2);
ms = optimvar('ms','LowerBound',0.5,'UpperBound',2);

x0.H = 1.5;
x0.l = 1;
x0.C = 0.3;
x0.M =1.2;
x0.D = 0.14;
x0.ms= 1.2;

volrazzo = H * D * D * pi*1.2;

[costo] = fcn2optimexpr(@ott2,H,l,C,M,D,ms);

rocketproblem = optimproblem('ObjectiveSense','max');

rocketproblem.Constraints.consv = volrazzo >= M/1000;
rocketproblem.Constraints.consm = M <= ms;
rocketproblem.Constraints.consmtot = M + ms <= 3;

rocketproblem.Objective = costo;

show(rocketproblem);

[sol,fval] = solve(rocketproblem,x0);
