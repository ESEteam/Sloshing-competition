function out = WRfn( Pi, V_h2o, H, Dcyl, Dnoz, mf, Hlan, tstep, z_step)


Hneck = H;          % altezza inizio restringimento bottiglia

D0 = Dcyl;         % diametro bottiglia subito prima dell'ugello
ks = 1.0;            % perdita carico H2O ugello circa 1.2
eta_air = 1.00;      % rendimento ugello 

% costanti del problema:
Ti = 298;
ro_h2o = 1000;

Patm = 1e5;
gam = 1.4;
g = 9.81;
R = 287;
RR = 8.31;
MM_air = 28.964e-3;

% variabili derivate:
Vtot = V(H);                                     % volume totale razzo
zi_h2o = fzero(@(z) V_h2o-V(z),0.5);             % altezza pelo libero iniziale
Ae = pi*Dnoz^2/4;                                % area ugello
mi_air = MM_air * Pi*(Vtot-V_h2o)/(R%sezione_tank = 2*pi*spess_tank*Dcyl;             % sezione approssimata serbatoio
%mf = mPay + V_h2o*ro_h2o + (sezione_tank * H) * ro_tank;      % massa a vuoto totale

% funzioni di z(t):
%A = @(z) pi*diam(z).^2/4;                       % area(z) (fase spinta acqua)
P = @(z,Piniz) Piniz *((Vtot-V_h2o)./(Vtot-V(z))).^gam;   % pressione(z) (spinta acqua)
P_lan = @(h) Pi .*(1+Ae.*h./(Vtot-V_h2o)).^(-gam);  % pressione durante il lancio con tubo
m = @(z) mf + ro_h2o*V(z);                       % massa totale razzo(z)
T = @(z,zp) ro_h2o*Ae*(diam(z)/Dnoz).^4.* zp.^2; % spinta ad acqua

%% spinta lanciatore


tspan = [0:tstep:1];
opts = odeset('RelTol',1e-10, 'AbsTol',1e-20, 'Refine', 10);
[t_lan,sol] = ode45(@(t,x) zpp(t,x), tspan, [0,0], opts);

z_lan = sol(:,1);
z_lan = z_lan(z_lan<= Hlan);
%zp_lan = sol(1:length(z_lan),2);
t_lan = t_lan(1:length(z_lan));

thrust_lan = (P_lan(z_lan)-Patm)*Ae;

P_lanEx = P_lan(z_lan(end));

%% spinta ad acqua

% risolvo l'eq diff.
tspan = [0:tstep:10];
opts = odeset('AbsTol',1e-15, 'Refine', 10);
[t1,z_sol] = ode45(@(t,z) -sqrt((2*(P(z,P_lanEx)-Patm)/ro_h2o + 2*g*z)/((diam(z)/Dnoz)^4*ks -1)) , tspan, zi_h2o, opts);

z_sol = z_sol(z_sol>=0); % levo i tempi successivi al burnout h2o
zp_sol =  -(((2*P(z_sol,P_lanEx)-Patm)./ro_h2o + 2*g.*z_sol)./((diam(z_sol)/Dnoz).^4*ks -1)).^0.5;
t1 = t1(1:length(z_sol)); % tempo fino al burnout h2o

% calcolo grandezze:
ro_air = mi_air./(Vtot - V(z_sol));
T_air = P(z_sol,P_lanEx)./(ro_air.* 287.1);
%P_air = P(z_sol);
thrust_h2o = T(z_sol,zp_sol);

%% spinta ad aria portata bloccata:

% per t > t_fine_acqua
%ro0_air2 = ro_air(end);
P0_air2 = P(z_sol(end),P_lanEx);
T0_air2 = T_air(end);
t2 = [0:tstep:2];

P_air2 = P0_air2*(1+Ae*(gam*R*T0_air2)^0.5./Vtot*(gam-1)/2*(2/(gam+1)).^((gam+1)/(2*(gam-1))).*t2).^((2*gam)/(1-gam));
T_air2 = T0_air2*(P_air2./P0_air2).^((gam-1)/gam);
ue_air2 = (eta_air * 2*gam/(gam+1)*R*T_air2 .* (1-(Patm./P_air2).^((gam-1)/gam)) ).^0.5;
mp_choked = (P_air2*Ae*gam*(2/(gam+1)).^((gam+1)./(2*(gam-1))))./(((gam*R*T_air2).^0.5));
thrust_air = mp_choked .* ue_air2;


% unifico i due tratti di spinta lancio + acqua + aria:
t2 = t2';
t_tot = [t_lan; t1 + t_lan(end)+tstep; t2 + t1(end) + t_lan(end)+tstep];

%ro = [ro_air; zeros(length(t2),1)];
%zp = [zp_sol; zeros(length(t2),1)];
z_tot = [(ones(length(t_lan),1).*zi_h2o); z_sol ; zeros(length(t2),1)];
%P_tot = [P_air; P_air2'];

%% cinematica:

thrust = real([thrust_lan; thrust_h2o; thrust_air']);

acc = thrust./m(z_tot) ;
% integro per le velocità:
vel = zeros(1,length(t_tot));
for temp=2:length(t_tot) 
   vel(temp) = trapz(t_tot(1:temp),acc(1:temp));
end

out.t = t_tot;
out.thrust = thrust;
out.vel = vel;
out.acc = acc;
out.m = m(z_tot);

%% funzioni

function D = diam(z)

    %global Hneck Dcyl D0

    D = zeros(length(z),1);
    for i=1:length(z)
        if z(i) <= Hneck
            D(i) = D0 - (Dcyl-D0)*0.5*(cos(pi*z(i)/Hneck)-1);
        else
            D(i) = Dcyl;
        end
    end

end

function Vol = V(z)

    %global z_step
    Vol = zeros(length(z),1);
    for i = 1:length(z)
        Vol(i) =  sum(z_step*pi*(diam([0:z_step:z(i)]').^2/4));
    end
    
end

 
function z_pp = zpp(t,x)
    z_pp(1) = x(2);
    z_pp(2) = (P_lan(x(1))-Patm)*Ae./m(zi_h2o);
    z_pp = z_pp';
end

end

