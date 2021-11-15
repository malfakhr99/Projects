clc;
clear all;
close all;

%%%GIVENS
D_out1 = 0.635;
D_out2 = 0.47625;
D_t = 4.445;
D_Md = 0.47625;
g = 981;            %cm/s^2
A = (pi/4)*D_t^2;   %All tank areas are the same (A11 = A12 = A21 = A22)
a = (pi/4)*D_Md^2;  %all tank outlet offices are the same (a11 = a12 = a21 = a22)

%%%EXPERIMENTAL DATA
%pump flow constants
kp11 = A*((25.4597-5.7014)/(27.466-7.842))/5;   %cm^3/(V*s)
kp12 = A*((21.1272-2.4112)/(27.448-5.798))/5;
kp21 = A*((6.1317-0.9284)/(27.464-10.128))/5;
kp22 = A*((2.597-0.3619)/(27.454-11.46))/5;
%sensor gains (for y=sensorgain*height)
sensorGain11 = (6/5.5 + 6.3/4.8)/2;             %h = (Sensor Height)cm/(Observed Height)cm (sensor gain)
sensorGain12 = (3.3/3.2 + 3.5/3.5)/2;
sensorGain21 = (9.2/12 + 9.6/11.9)/2;
sensorGain22 = (3.5/5.2 + 3.9/5)/2;
%desired equilibrium height (current values are steady state values at 1p05 2p05)
ho11 = 5.0;     
ho12 = 3.0;
ho21 = 12.0;
ho22 = 5.5;

%%%STATE SPACE
c11 = kp11/A;
c12 = kp12/A;
c21 = kp21/A;
c22 = kp22/A;
b=(a/A)*sqrt(2*g);
el_11 = (1/2)*b/sqrt(ho11);
el_12 = (1/2)*b/sqrt(ho12);
el_21 = (1/2)*b/sqrt(ho21);
el_22 = (1/2)*b/sqrt(ho22);

A = [-el_11    0       0       0  ;
       0    -el_12     0       0  ;
     el_11     0    -el_21     0  ;
       0     el_12     0    -el_22];  
B = [c11  0 ;
      0  c12;
      0  c21;
     c22  0 ];
C = [0 0 sensorGain21      0      ;
     0 0     0        sensorGain22];
D = [0 0;
    0 0];
s=tf('s');
sI= [s 0 0 0;
     0 s 0 0;
     0 0 s 0;
     0 0 0 s];
ytf = C*inv(sI-A)*B;

%inputs are columns, outputs are rows, so its (rows,cols)
ytf(1,1); %input1 output1 = h21 2nd order
ytf(1,2); %input2 output1 = h11
ytf(2,1); %input1 output2 = h12
ytf(2,2); %input2 output2 = h22 2nd order

PlantStatic = evalfr(ytf,0);
DecoupleStatic = pinv(PlantStatic)*C;

decoupledShrink = ones(2,2);

decoupledShrink(1,1) = DecoupleStatic(1,3);
decoupledShrink(1,2) = DecoupleStatic(1,4);
decoupledShrink(2,1) = DecoupleStatic(2,3);
decoupledShrink(2,2) = DecoupleStatic(2,4);
D = decoupledShrink;
initialC = C;
resultantC = decoupledShrink*PlantStatic;

phat = D*ytf;

OLsys1 = tf([8.388/12*0.17285^2],[1 2*0.55593*0.17285 0.17285^2]);  % https://electronics.stackexchange.com/questions/117124/how-do-i-find-the-second-order-transfer-function-from-this-step-response-diagram
OLsys2 = tf([3.254/5.5],[5.411 1]);

CLsys1 = -OLsys1/(OLsys1-1);    % used as model transfer functions
CLsys2 = -OLsys2/(OLsys2-1);

P1 = pole(CLsys1)   % used for IMC Controller (inverse the poles & zeros)
Z1 = zero(CLsys1)
P2 = pole(CLsys2)
Z2 = zero(CLsys2)








%%% NOT USED

% Model21 = tf([28.18/12],[23.5 1]);   % determined by closed-loop first-order simplification of Plant
% Model22 = tf([8.814/5.5],[15.2 1]);
% 
% P21 = pole(Model21) % used for IMC controller
% Z21 = zero(Model21)
% P22 = pole(Model22)
% Z22 = zero(Model22)
