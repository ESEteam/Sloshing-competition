function [costo] = beccstatico(d2,d3,cs,flagplot)
% -----------------------\
% stab          ala        \
% ---- slosh   -----        |
% -> x=0                   /
% -----------------------/
%% dati
g=9.81;
ro = 1.225;        
roH2o=1000;
Hsl=0.27     ;      %altezza serbatoio slosh
Dserbatoiosl=0.04;  %diametro serbaoiosloshing
alphast=25;         %angolo di stallo

%% posizioni

cw = 0.2;               %corda ala
% cs = 0.2;     %corda stabilizzatore
Cm0 = 0.08;       %coefficiente di momento
v=10;     %velocità verticale
ARw=5;
         %apertura alare
         %apertura stabilizzatore
         
%% masse

M=1.0;            % massa a vuoto
Mala=0.4;
Mstab=0.3;
Msl=1.0+0.5;        % massa di sloshing
Mtot=M+Msl+Mala+Mstab;     % massa totale


d1 =1.3;                                % distanza dalla base del bordo d'uscita dello stabilizzatore  
% d2 = 0.6;                                % distanza dalla base del bordo d'uscita dell'ala
%d3 = linspace(d2+cw*3/4,1.2);                     % distanza dalla base della base bassa del serbatoio di sloshing
Xg = 0.7;       % posizione del centro di massa a vuoto (senza sloshing)% deltaMsl=(Hsl^2*tan(alphast*pi/180)/8)*roH2o*Dserbatoiosl;
deltaMsl=(Hsl^2*tan(alphast*pi/180)/8)*roH2o*Dserbatoiosl;

Gsl=(2/3*Hsl*deltaMsl/Msl)                ;            %baricentro dell'acqua nel serbatoio allo stallo
 Xgsl = [d3+Hsl/2-Gsl ; d3+Hsl/2 ; d3+Hsl/2+Gsl];           % posizione centro di massa slosh

% Xgsl=[d3+Hsl/4; d3+Hsl/2; d3+3/4*Hsl];
%% parametri ala e stabilizzatore 

Sw=cw*ARw*cw;

%% stabilità 

dg = (Xgsl*Msl+Xg*M+(d2+cw/2)*Mala+(d1+cs/2)*Mstab)/Mtot;        %posizione del centro di massa totale
stabilita = dg - (d2+3*cw/4);     %distanza tra cp e dg, stabile se stabilità > 0

%bracci

braccioLw= d2 + 3*cw/4 - dg;
braccioLs= d1 + 3*cs/4 - dg;

%equilibrio a momento
% Cms=[-0.08 0 -0.08];
% Cls=[-1.07 -0.61 -1.07];     % centrale prifilo a -5gradi e flap a 0 gradi;
Cms=+0.08;
Cls=+1.07;
% f = (0.5*ro*(v^2)*(Cm0*Sw + Cms*Ss) - braccioLw*Mtot*g )./(braccioLs-braccioLw);

Ss= (2*braccioLw*Mtot*g/(ro*(v^2))-Sw*Cm0)./(Cms-Cls*(braccioLs-braccioLw));

Ls = 0.5*ro*v^2*Ss.*Cls;

%ipotesi di accelerazione verticale quasi nulla,per trovare valore di Lw,Ls

% Cls=1;   % con incidenza 5 gradi e flap deflesso di 10 gradi
% eta=abs(Ls./(0.5*v^2*cs^2*Cls));
% lstab1=eta*cs+((eta*cs).^2+8*eta*cs.^2).^0.5/2;


% 
if flagplot == 1
    
figure;
subplot(2,2,1)
plot(d3,stabilita,'x')
xlabel('posizione serbatoio slosh')
ylabel('distanza tra cp e cg (>0 stabile)')
legend('muso alto','muso dritto','muso basso')
yline(cw/8)
legend('muso alto','muso dritto','muso basso','corda ala/4')

Ss
lstab=Ss/cs
Ls
dg
% % subplot(2,2,2)
% % plot(dg,Ls(),'o');
% % yline(0);
% % xlabel('posizione serbatoio slosh')
% % ylabel('portanza stabilizzatore N')
% % legend('muso alto','muso basso','0 N')
% % 
% % % dimensioni stabilizzatore
% % subplot(2,2,3)
% % 
% % 
% % plot(d3,Ss/cs,'o')
% % legend('muso alto','muso dritto','muso basso');
% % yline(1)
% % 
% % subplot(2,2,4)
% % 
% % plot(min(stabilita)-cw/8,'x');
% % 
end


if (min(stabilita)-cw/8) < 0
   costo=inf;
else

  costo = abs(max(Ls));
end



end
