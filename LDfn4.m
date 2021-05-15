function [L,D]=LDfn4(alfa, U, cyl, wing, stab, Slift, rktType) 
% funzione per calcolare portanza e attrito data la geometria del razzo
% ipotesi di semplificazione:
% 
% dati in ingresso:
% cyl.H, cyl.D
% stab.L, stab.Cave
% wing.L, wing.Cr, wing.rastr

% parametri stimati a partire dai dati in ingresso alla funzione:
cyl.Hnosecone = 1.5*cyl.D;
cyl.Dmin = 2*cyl.Dnoz;
cyl.HnozTaper = 0.1* cyl.H;
wing.Slift = Slift;
wing.thick = wing.Cr* 0.01;
wing.Cave = wing.Cr*(1+wing.taper)/2;
stab.thick = stab.Cr* 0.;
stab.Cave = stab.Cr*(1+stab.taper)/2;
stab.Slift = 2*stab.L*stab.Cave;

nuAir = 1.5111e-5;
roAir = 1.225;

%wing.NACA = ["6409", "0012", "9301"];    % 6409 per la config classica; 0012 e 9301 rispett. per la conf chiusa e aperta 
sigma=0.4;                                % correzione CdInd ala rastremata

% parametri calcolati:
cyl.Hbody = cyl.H-(cyl.Hnosecone+cyl.HnozTaper);
Cf = @(l) 1.328/(U*l/nuAir)^0.5;                                   %coeff. attrito viscoso

%% calcolo i vari casi:

if rktType==3 || rktType==1 % razzo ala aperta o ala classica
    AR = (cyl.D + 2*wing.L)/wing.Cave;
    
    if(rktType==3) % ala aperta
        % parametri profilo e377m (tipo NACA9301):
        Clalfa  = 0.1;
        alfa0 = -4*pi/180; %[deg]
        CdAtt = 0.08;
    else % ala classica
        % parametri profilo SD7032:
        Clalfa  = 0.11;
        alfa0 = -2.5*pi/180; %[deg]
        CdAtt = 0.04;
    end
    
    %% portanza
    
    Clfoil = Clalfa*(alfa - alfa0*180/pi);                         %Cl profilo (alfa in deg)
    Cl_W   = Clfoil*pi*AR/(pi*AR + Clalfa);                        %Cl ala
    Cn_Cyl = cyl.D*cyl.H/(cyl.D^2*pi/4) * (alfa*pi/180)^2;
    
    Nbody  = 0.5 * roAir * U^2 * (cyl.D*cyl.H) * Cn_Cyl;
    Lw = 0.5 * roAir * U^2 * (wing.Slift) * Cl_W;
    
    L = Nbody * cos(alfa*pi/180) + Lw;
    
    %% attrito
    Cd0_Wind   = (1+sigma)*(Cl_W^2)/(AR*pi);                    %Cd indotto ala circa
    Cd0_Wattr  = CdAtt;                                         %Cd attrito profilo per alfa < 13deg
    %Cd0_Wattr = 2*Cf(wing.L) * (1 + 2*wing.thick/wing.L) * 4* (Slift)/(pi*cyl.D^2);
    Cd_W = Cd0_Wind + Cd0_Wattr;                                % ala non distacca lo strato limite fino a 10deg
    
    Cd0_Cyl   = Cf(cyl.H)*(1 + 60/(cyl.H/cyl.D)^3 + 0.0025*cyl.Hbody/cyl.D) * ( 2.7*cyl.Hnosecone + 4*cyl.Hbody + 2*cyl.HnozTaper*(1-cyl.Dmin/cyl.D) )/cyl.D;
    Cd0_Base   = 0.029 * (cyl.Dmin/cyl.D)^3/(Cd0_Cyl)^0.5;
    Cd0_Interf = 2*Cf(cyl.H)*(1 + 2*wing.thick/wing.L)*4*2*wing.Cr/(2*pi*cyl.D);
    
    delta = 0.7; % circa sperimentale
    eta   = 0.6; % circa sperimentale
    Cd0 =  Cd0_Cyl + Cd0_Base + Cd0_Interf;
    Cd_Body = Cd0 + 2*delta* (alfa*pi/180)^2 + (alfa*pi/180)^3 * 3.6*eta*(1.36*cyl.H - 0.55*cyl.Hnosecone)/(pi*cyl.D); % Cd corpo al variare di alfa
    
    Dw    = 0.5 * roAir * U^2 * (wing.Slift + stab.Slift) * Cd_W;
    Dbody = 0.5 * roAir * U^2 * cyl.D^2*pi/4 * Cd_Body;
    
    D = Dw + Dbody + Nbody * sin(alfa*pi/180) ;
    
else % razzo ala chiusa
    AR = (cyl.D + 2*stab.L)/stab.Cave;
    
    % parametri profilo e377m (tipo NACA9301):
    Clalfa  = 0.11;
    alfa0 = 0; %[deg]
    CdAtt = 0.04;

    %% portanza
    
    Clfoil = Clalfa*(alfa - alfa0*180/pi);                         %Cl profilo (alfa in deg)
    Cl_W   = Clfoil*pi*AR/(pi*AR + Clalfa);                        %Cl ala
    Cn_Cyl = cyl.D*cyl.H/(cyl.D^2*pi/4) * (alfa*pi/180)^2;
    
    Nbody  = 0.5 * roAir * U^2 * (cyl.D*cyl.H) * Cn_Cyl;
    Lw = 0.5 * roAir * U^2 * stab.Slift * Clfoil;
    
    L = Nbody * cos(alfa*pi/180) + Lw;
    
    %% attrito
    Cd0_Wind   = (1)*(Cl_W^2)/(AR*pi);                     %Cd indotto ala circa
    Cd0_Wattr  = CdAtt;                                         %Cd attrito profilo per alfa < 13deg
    %Cd0_Wattr = 2*Cf(stab.L) * (1 + 2*stab.thick/stab.L) * 4* (Slift)/(pi*cyl.D^2);
    Cd_W = Cd0_Wind + Cd0_Wattr;                                 % ala non distacca lo strato limite fino a 10deg
    
    Cd0_Cyl   = Cf(cyl.H)*(1 + 60/(cyl.H/cyl.D)^3 + 0.0025*cyl.Hbody/cyl.D) * ( 2.7*cyl.Hnosecone + 4*cyl.Hbody + 2*cyl.HnozTaper*(1-cyl.Dmin/cyl.D) )/cyl.D;
    Cd0_Base   = 0.029 * (cyl.Dmin/cyl.D)^3/(Cd0_Cyl)^0.5;
    Cd0_Interf = 2*Cf(cyl.H)*(1 + 2*stab.thick/stab.L)*4*2*stab.Cr/(2*pi*cyl.D);
    
    delta = 0.7; % circa sperimentale
    eta   = 0.6; % circa sperimentale
    Cd0 =  + Cd0_Cyl + Cd0_Base + Cd0_Interf;
    Cd_Body = Cd0 + 2*delta* (alfa*pi/180)^2 + (alfa*pi/180)^3 * 3.6*eta*(1.36*cyl.H - 0.55*cyl.Hnosecone)/(pi*cyl.D); % Cd corpo al variare di alfa
    
    Dw    = 0.5 * roAir * U^2 * stab.Slift * Cd_W;
    Dbody = 0.5 * roAir * U^2 * (cyl.D)^2*pi/4 * Cd_Body;
    
    D = Dw + Dbody + Nbody * sin(alfa*pi/180) ;
    % ala
end

end
