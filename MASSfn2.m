% MASS fnc per entrambi i tipi di razzo 1 --> ala fissa; 2--> ala mobile

function out = MASSfn2(cyl, spar, wing, stab, ms, varieFlag, rktType)
% cyl.H, cyl.D, cyl.Nwrap
% spar...
% stab.L, stab.Cave, stab.Nstri
% wing.L, wing.Cr, wing.rastr, wing.Nribs, wing.Nstri, wing.letBool

%% potenzialmente fissi:
% Hcyl = 1.4;
% rastrWing = 0.4; 
% Hspar = 0.5;
% Dspar = 0.04;
 spar.Nwrap = 2;
 %cyl.Nwrap  = 3;
 
 %wing.Nlay  = 2;
 wing.Nrods = 0;                % asticelle retrattili in totale
 wing.Nribs = wing.L/0.3;
 wing.Nstri = wing.Cr/0.15;     % numero stringers sulle ali e stabilizzatore
 stab.Cave = stab.Cr *(1+stab.taper)/2;
 stab.Nstri = stab.Cave/0.1;
 
 roCarb  = 0.16;                % densita per mq
 roGlass = [0.05, 0.162];       % densita per mq
 roRods  = [0.01,0.025];         % massa per m

if rktType == 1 
    %% dati assunti noti dalla funzione:
    
    %roEpox = 1000;
    percEpox = 150;                             % percentuale epox in massa rispetto al tessuto
    Swinglet = pi* (wing.Cr*wing.taper)^2/4 *2; %superficie winglet forma cerchio approx
    NlayerRib = 3;                              % strati carbonio per i profili
    kAveRib = 0.063;                            % coeff proporz. fra corda_media^2 e superficie_profili
    kCorrWing = 1.05;                           % maggiorazione della superficie alare dovuta a spessore ala
    Hstri = wing.L;                             % lunghezza stringers
    HstriStab = stab.L;
    Mvarie = 0.3;                               % masse sensori motori e controllo
    
    %% calcolo massa 1 ALA:
    
    Cave = wing.Cr*(wing.taper+1)*0.5;
    Swing = Cave*wing.L;                        % superficie portante ala rastremata (trapezio)
    Sstab = stab.Cave * stab.L;                 % superficie stabilizzatore
    Sskin = (Swing + Sstab) * 2 * kCorrWing;      % superficie skin
    SribAve = Cave^2 * kAveRib;                 % superficie media profili
    Slift = Swing; % + Sstab*0.7;
    
    Mskin = wing.Nlay * max(roGlass)*Sskin;
    Mribs = roCarb*SribAve*NlayerRib*wing.Nribs;
    Mstri = max(roRods)*( Hstri*wing.Nstri + HstriStab*stab.Nstri) ;
    Mwinglet = Swinglet*min(roGlass);
    Mspar = spar.Nwrap*pi*spar.D*spar.H*roCarb;
    
    Mwing = (1 + percEpox/100)*(Mskin + Mribs + Mwinglet*wing.letFlag + Mspar) + Mstri;
    
    %% calcolo massa cilindro:
    
    Mcyl = (1 + percEpox/100)*(cyl.D*pi*(cyl.H*1.1)*roCarb*cyl.Nwrap);              

    out.Mdry = 2*Mwing + Mcyl + Mvarie*varieFlag;
    out.Mwing_stab = Mwing;
    out.Mcyl = Mcyl;
    out.Slift = 2*Slift;
    
else
    
    percEpox = 110;                                                                 % percentuale epox in massa rispetto al tessuto
    Mvarie = 0.4;                                                                   % masse sensori motori e controllo
    
    Mcyl  = (1 + 2*percEpox/100)*(cyl.D*pi*cyl.H*roCarb*cyl.Nwrap);             
    Mstab = wing.Nlay * max(roGlass)*(stab.Cave * stab.L)*3;  
    Ltank = (ms/1000 *2)/(((cyl.D + 0.05)^2 - cyl.D^2)*pi/4);                       % lunghezza serbatoio slosh
    Mtank = Ltank*1.4 * 0.001*pi*(2*cyl.D + 0.07) * 1250;
    Mrods = (1 + percEpox/100)*max(roGlass)*wing.Nlay*(wing.L * wing.Dba*pi) + ...  % massa asta del bordo attacco
            wing.Nrods*wing.Cr*min(roRods) + wing.Ltip*max(roRods);                 % massa asticelle retrattili e asta in punta dell'ala
    Mstri = min(roRods)*( stab.L*stab.Nstri) ;
    Swing = (wing.Cr + wing.Ltip/1.4)*(wing.L)/2 ;
    Mtelo = min(roGlass) * Swing;
    
    Mwing = (1 + percEpox/100)*(Mstab) + Mrods + Mtelo + Mstri;
    
    out.Mdry = 2*Mwing + Mcyl + Mvarie*varieFlag;
    
    2*Mwing + Mcyl + Mtank + Mvarie*varieFlag;
    out.Mwing_stab = Mwing;
    out.Mcyl = Mcyl;
    out.Mvarie = Mvarie;
    wing.Slift = 2*(Swing); % + 0.8*stab.Cave*stab.L);
    out.Slift = wing.Slift;
    
end
    
end
