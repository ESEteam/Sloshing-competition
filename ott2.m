function [punt] = ott2( H,l,C,Mh2o,Dcyl,mslosh, Lstab, Cstab, plotFlag)
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
L = zeros(1, zeri);
D = zeros(1, zeri);


%%  Variabili

tstep  = 350;
tstep2 = 1/tstep;
z_step = 1e-2;

%% variabili fisse e non
Pi = 11e5;
Hlanc = 2.8;                                    % H tubo di lancio
g  = 9.81;

aref = 0;   % negativo verso il basso: angolo fra l'orizzontale e l'asse del razzo: ipotizzato costante per ora

% masse:
cyl.H = H;                                      % altezza bottiglia
cyl.D = Dcyl;                                   % diametro razzo
cyl.Dnoz = 30e-3;                               % diametro ugello
cyl.Nwrap  = 4;                                 % numero strati tessuto

stab.L     = Lstab;
stab.Cr    = Cstab; 
stab.taper = 0.7;                               % rapporto rastremazione
 
wing.L     = l;                                 % lungh asta retrattile al B.A.
wing.Cr    = C;                                 % corda di radice (per corda di radice si intende distanza fra l'asta retrattile e il B.A. dello stab)
wing.taper = 0.9;
wing.Nlay  = 3; 

spar.H = 0.01;
spar.D = 0.04;

wing.Dba   = 0.03;                              % diametro asta retrattile bordo attacco
wing.Ltip  = 0.6*l;                             % lunghezza asta in punta dell'ala

wing.letFlag = 1;                               % 1--> include massa winglet
varieFlag = 1;                                  % 1--> include massa servi,sensori,controlli + giunti vari + meccanica

rocketType = 2;                                 % 1--> classico; 2--> ali retrattili_chiuse; 3 --> ali retrattili aperte: wing.NACA = [6409, 0012, 9301]

%%  inizializzazione

V_h2o = Mh2o/1000;
M = MASSfn2(cyl, spar, wing, stab, mslosh, varieFlag, rocketType);
mf = M.Mdry + mslosh;

out = WRfn( Pi, V_h2o, cyl.H, cyl.D, cyl.Dnoz, mf, Hlanc, tstep2, z_step);

m = out.m; 
T = out.thrust';
lent = length(T);
lenm = length(m);

vin= 1;   
ai = 82;   % angolo di lancio
x(1)=0;
y(1)=0.5;
U(1)= vin;
alfa(1) = ai*pi/180; % angolo della velocità rispetto all' orizzotale
L(1)=0;
D(1)=0;
ax(1)=0;
ay(1)= 0;
vx(1)=vin*cos(ai*pi/180);
vy(1)=vin*sin(ai*pi/180);

i=2;
im1=1;
im2=0;
LiftFlag = 0;

while  y(im1)>=0
    im1=i-1;
    im2 = i-2;
    
    U(i)=( vx(im1)^2 + vy(im1)^2)^0.5;
    alfa(i)= atan2(vy(im1),vx(im1));
    
    if(vy(im1)>0)
        gamma(i)   = 0;                     % incidenza (fra velocita e asse razzo)
        rocketType = 2;                     % ala chiusa
        %LiftFlag=0; %% perche non c'era? 
    else
        gamma(i)= aref - alfa(i)*180/pi;    % incidenza in discesa (alfa<0 --> incidenza positiva) 
        LiftFlag=1;
        rocketType = 3;                     % ala aperta
        
        if abs(gamma(i))>5
            gamma(i)=5;                    % se vy è troppo bassa impongo una incidenza costante di 10deg
        end
    end
    
    % azzero spinta e massa dopo fine spinta ad aria per sicurezza:
    if(i>lent-1)
        T(im1)=0;
    end
    
    if(i>lenm-1)
        m(i)=m(lenm-10);
    end
    
    [L(i),D(i)] = LDfn4(gamma(i), U(i), cyl, wing, stab, M.Slift, rocketType); % (alfa(deg), U, ....) 
    ax(i) = (1/m(i))* ( - D(i)*cos(alfa(i)) - LiftFlag*L(i)*sin(alfa(i)) + T(im1)*cos(alfa(i)) );
    ay(i) = (1/m(i))* ( - D(i)*sin(alfa(i)) + LiftFlag*L(i)*cos(alfa(i)) + T(im1)*sin(alfa(i)) -m(i)*g );
    
    vy(i)= vy(im1) + ay(i)*(1/tstep);
    vx(i)= vx(im1) + ax(i)*(1/tstep);
    x(i)= x(im1) + vx(i)*(1/tstep);
    y(i)= y(im1) + vy(i)*(1/tstep);
    
    i=i+1;
end

t_end=length(y)/tstep;

punt = (t_end+x(im1))/m(1)*(mslosh) ; %punteggio da massimizzare
effic = L(end)/D(end);

if plotFlag==1
    tPlot = linspace(0,t_end,length(y));
    
    figure(1)
    
    subplot(3,3,1)
    plot(tPlot,x)
    legend('t-x')
    subplot(3,3,2)
    plot(tPlot,y)
    legend('t-y')
    subplot(3,3,3)
    plot(x,y)
    legend('x-y')
    
    subplot(3,3,4)
    plot(tPlot,vx)
    legend('t-vx')
    subplot(3,3,5)
    plot(tPlot,vy)
    legend('t-vy')
    subplot(3,3,6)
    plot(tPlot,U)
    legend('t-absV')
    
    subplot(3,3,7)
    plot(tPlot,ax)
    legend('t-ax')
    subplot(3,3,8)
    plot(tPlot,ay)
    legend('t-ay')
    subplot(3,3,9)
    plot(tPlot,(ax.^2+ay.^2).^0.5)
    legend('t-absAcc')    
    
    figure (2)
    plot(tPlot,gamma)
    legend('t-incidenza')
end

end

