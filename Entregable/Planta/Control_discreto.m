clear all
close all
clc

%% Identificación por IDENT

% load datos_OM.mat
% Gs = tf(Gs.Numerator,Gs.Denominator);

load datos_KJ.mat
Gs = tf(Gs.Numerator,Gs.Denominator);

disp(Gs)


polos = pole(Gs);
ceros = zero(Gs);
num = poly(ceros);
den = poly(polos);


%[A,B,C,D] = tf2ss(cell2mat(Gs.Numerator),cell2mat(Gs.Denominator));

A = [0 1; -0.2546 -0.3939]
B = [0 ; 1]
C = [0.2506 0]
D = 0

figure(1)
step(Gs)
disp(Gs)

%% Control retroalimentación de estados - Continuo

Mp_des = 0.05;
ts_des = 7;
zita_des = sqrt(((log(Mp_des))^2)/(((log(Mp_des))^2)+pi^2));
wn_des = 4/(ts_des*zita_des);

polosCont = roots([1.,2*zita_des*wn_des,wn_des^2]);
disp(polosCont)

SysDis = c2d(Gs, 0.004);
discreto = 1;
if(discreto == 0)
    [Adis, Bdis, Cdis, Ddis] = tf2ss(cell2mat(SysDis.Numerator),cell2mat(SysDis.Denominator));
else
    Adis = A;
    Bdis = B;
    Cdis = C;
    Ddis = D;
end

A_c = Adis;
B_c = Bdis;

if (discreto == 0)
    Kcont = place(A_c,B_c,exp(polosCont*0.004));
else
    Kcont = place(A_c,B_c,polosCont);
end

disp(Kcont)


Acontrolado_Cont = (A_c - mtimes(B_c,Kcont));
Bcontrolado_Cont = B_c;

Ccont_Cont = Cdis;
Dcont_Cont = 0;

if(discreto == 0)
    sysControlado = ss(Acontrolado_Cont,Bcontrolado_Cont,Ccont_Cont,Dcont_Cont,0.004);
else
    sysControlado = ss(Acontrolado_Cont,Bcontrolado_Cont,Ccont_Cont,Dcont_Cont);
end

% Definir el vector de tiempo
tiempo = 0:0.004;35;

% Definir la entrada de control
U = ones(1, length(tiempo));

% Calcular la respuesta al escalón del sistema controlado
[youtContr, Tcont] = step(sysControlado, 35);
figure(4)
plot(Tcont, youtContr, 'DisplayName', 'yCont con control u=-kx');
hold on;
step(Gs,'r')%, '-.', 'DisplayName', 'yCont sin control');
hold off;
legend;
grid on;
xlabel('Tiempo');
ylabel('Salida');
title('Comparación de salida con y sin control');

%% Observador

Polos_obs = [-3,-3.5];

% rp = real(polosCont);
% op1 = round(rp(1))*10;
% op2 = op1-1;
% Polos_obs = [op1,op2];

A_ob = Adis';
C_ob = Cdis';

if(discreto == 0)
    k_obs = place(A_ob,C_ob,exp(Polos_obs*0.004));
else
    k_obs = place(A_ob,C_ob,Polos_obs);
end

k_obs = k_obs';
disp(k_obs)

%% DataLabview

load datosPlanta_LABVIEW.mat

