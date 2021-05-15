close all
clear
clc
cw=0.2;
costomin= inf;
for d2 = 0.5:0.01:1.3
%     for d3 =  0.2:0.01:1.1
        for cs = 0.1:0.005:0.15
        d3=d2+cw;
                 costo= beccstatico(d2,d3,cs,0);
                 
                
                if( costo <= costomin)
                    costomin = costo;
                    
                    O.d2=d2;
                    O.d3=d3;
                    O.cs=cs;
                  
                    
                    
                end
                
             end
        end
% end


beccstatico(O.d2,O.d3,O.cs,1);



