function [punt] = ott2( H,l,C,M,Dcyl,mslosh)
%% Zeri

zeri = 5000;


U = zeros(1, zeri);
alfa = zeros(1, zeri);
gamma = zeros(1, zeri);
x = zeros(1, zeri);
y = zeros(1, zeri);
vx = zeros(1, zeri);
vy = zeros(1, zeri);
ax = zeros(1, zeri);
ay = zeros(1, zeri);
cd = zeros(1, zeri);
cdc = zeros(1, zeri);
cl = zeros(1, zeri);
L = zeros(1, zeri);
D = zeros(1, zeri);


%%  Variabili
dat=importdata('naca9301.dat');
% H, S , V_h20, Dcyl
tstep=1000;

% variabili per wrfn
Pi = 11e5;
Dnoz = 20e-3;                                   % diametro ugello
Hlanc = 2.7;                                    % H tubo di lancio
tstep2 = 1/tstep;
z_step = 1e-2;

% variabili per simulazione volo

vin=1;   % dovrebbe essere a 0
alfa0= -2;
aref=0;   % il meno è verso il basso

% cl0=0.3;
% clm= 0.085;
% cdcl = 15;
% cdc = 1.3;
ro=1.225;
g= 9.81;
ai=85;   % angolo di lancio

R = 0.3;


%% masse :
cyl.H = H;
cyl.D = Dcyl;
cyl.Nwrap = 2;                                  % numero di giri di tessuto

spar.H = 0.5;
spar.D = 0.04;
spar.Nwrap = 2;

wing.L = l;
wing.Cr = C;                                    % corda di radice
wing.rastr = R;                               % rapporto rastremazione
wing.Nribs = 6;                                 % numero ribs
wing.Nstri = 3;                                 % numero stringers
wing.letBool = 1;                               % 1--> include massa winglet

varieBool = 1;                                  % 1--> include massa servo,controlli + giunti ali/corpo

roCarb = [0.16, 0.2];                           % densita per mq
roGlass = [0.05, 0.16];                         % densita per mq
roStri = 0.044;                                 % densita per m
%%  inizializzazione

S = (C + C*R)*l;
V_h2o=M/1000;

M = MASSfn(cyl, spar, wing, roCarb, roGlass, roStri, varieBool);
m = M.Mtot + mslosh;

out = WRfn( Pi, V_h2o, H, Dcyl, Dnoz, m, Hlanc, tstep2, z_step);

m = out.m;
T = out.thrust';
lent=length(T);
lenm=length(m);

x(1)=0;
y(1)=0.5;
U(1)= vin;
alfa(1)=ai*pi/180;
L(1)=0;
D(1)=0;
cd(1)=0;
cdc(1)=0;
cl(1)=0;
ax(1)=0;
ay(1)= 0;
vx(1)=vin*cos(ai*pi/180);
vy(1)=vin*sin(ai*pi/180);


i=2;
k=1;
noL = 0;

while  y(k)>=0
    k=i-1;
    
    U(i)=( vx(k)^2 + vy(k)^2)^0.5;
    alfa(i)= atan2(vy(k),vx(k));
    
    if(vy(k)>0)
        gamma(i)=alfa0;
    else
        gamma(i)= aref - alfa(i)*180/pi;
        noL=1;
        
        if abs(gamma(i))>10
            gamma(i)=10;
        end
    end
    
    if(i>lent-1)
        T(k)=0; 
    end
    
     if(i>lenm-1)
        m(i)=m(lenm-10); 
    end
    
    [cl(i),cd(i),cdc(i)]= clcd(gamma(i),alfa0,Dcyl,H,l,C,dat,0);  
    
    L(i) = 0.5 * S * ro * U(i)^2 * cl(i) * noL;
    D(i) = 0.5 * ro * U(i)^2 * ( cd(i) * S + cdc(i) * ( Dcyl^2 * pi/4 )); % da aggiungere inclinazione fusoliera
    ax(i) = (1/m(i))* ( - D(i)*cos(alfa(i)) - L(i)*sin(alfa(i)) + T(k)*cos(alfa(i)) );
    ay(i) = (1/m(i))* ( - D(i)*sin(alfa(i)) + L(i)*cos(alfa(i)) + T(k)*sin(alfa(i)) -m(i)*g );
    
    vy(i)= vy(k) + ay(i)*(1/tstep);
    vx(i)= vx(k) + ax(i)*(1/tstep);
    x(i)= x(k) + vx(i)*(1/tstep);
    y(i)= y(k) + vy(i)*(1/tstep);
   
    i=i+1;
    
end

t=length(y)/tstep;

punt = (t+x(k))/m(1)*(mslosh); %reciproco del punteggio così posso minimizzare


end

