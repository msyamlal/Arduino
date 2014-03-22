%Lab4


we =600    %weight of the elevator (Kg)
me=1240  %mass of the elevator(with people+extra load)(Kg)
cabarea = .6*pi %cable cross section area(m^2)
E = 210*10^9 %youngs modulus
niu =0.3 %Poisson Ratio
G=150, %strain gauge factor
Res=300 %resistance of the strain gauge
Vs = 10 %supply voltage
Tempvar = 1/50 %temp change 1 deg per 50m
Tempsen = 1 %temperature sens. Coeff. 1% per degC for deltaR/R


timeforforce=[0 5 5.1 5.2 5.5 5.6 10 10.1 10.2 10.5 10.6 15];
inputforce=[0 0 me 2*me 2*me 0 0 -me -2*me -2*me 0 0];

