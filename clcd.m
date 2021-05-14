function [Cl,Cd,Cdc,AR]=clcd(teta0,alfa,D,H,L,Cradice,dat,I) %%RR=rapporto di rastremazione
RR=0.3;
teta=teta0-alfa;
xc=dat.data(:,1);
yc=dat.data(:,2);

Cmedia=Cradice*(1+RR)/2;
                       
 AR=2*L./Cmedia;

%profili sottili
sigma=0.2;

dy=diff(yc)./diff(xc);
dy(end)=[];
xc(end-1)=[];
xc(end)=[];

if(I==1)
alpha0=(2/pi)*trapz(xc,dy.*(xc./(1.000001-xc).^0.5));       %integrale teoria dei profili sottili per calcolo di alpha0
else 
alpha0=0;
end

Clfoil=0.11*(teta-(alpha0*180/pi));                        %Cl prolifilo
Cl=Clfoil*AR/(AR+2);                                        %Cl ala
 Cd=(1+sigma)*(Clfoil^2)/(AR*pi);                              %non considero la resistenza del profilo quindi uso il clfoil che

cdcfrontale = 0.65;                                
CdcTrasversale =1.3;   


Cdc = cdcfrontale*(cos(teta0*pi/180)^2) + CdcTrasversale*(sin(teta0*pi/180)^2)*4*H/(pi*D);

end