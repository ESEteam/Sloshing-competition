function out = MASSfn(cyl, spar, wing, roCarb, roGlass, roStri, varieBool)

%% dati assunti noti dalla funzione:

percEpox = 100;                             % percentuale epox in massa rispetto al tessuto
Swinglet = pi* (wing.Cr*wing.rastr)^2/4 *2; %superficie winglet forma cerchio approx
NlayerRib = 4;                              % strati carbonio per i profili
kAveRib = 0.063;                            % coeff proporz. fra corda_media^2 e superfcie_profili
kCorrWing = 1.02;                           % maggiorazione della superficie alare dovuta a spessore profili
Hstri = 0.8 * wing.L;                       % lunghezza stringers 
Mvarie = 0.3;                         % masse sensori motori e controllo

%% calcolo massa 1 ALA:

Cave = wing.Cr*(wing.rastr+1)*0.5;
Slift = Cave*wing.L;                        % superficie portante ala rastremata (trapezio)
Sskin = Slift * 2 * kCorrWing;              % superficie skin
SribAve = Cave^2 * kAveRib;                 % superficie media profili

Mskin = max(roGlass)*Sskin;
Mribs = min(roCarb)*SribAve*NlayerRib*wing.Nribs;
Mstri = roStri*Hstri*wing.Nstri;
Mwinglet = Swinglet*min(roGlass);
Mspar = spar.Nwrap*pi*spar.D*spar.H*min(roCarb);

Mwing = (1 + percEpox/100)*(Mskin + Mribs + Mwinglet*wing.letBool + Mspar) + Mstri;


%% calcolo massa cilindro:

Mcyl = (1 + percEpox/100)*(cyl.D*pi*cyl.H*max(roCarb)*cyl.Nwrap);

out.Mwing = Mwing;
out.Mcyl = Mcyl;
out.Slift = Slift;
out.Mtot = 2*Mwing + Mcyl + Mvarie*varieBool;

end
